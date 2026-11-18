#!/usr/bin/env python3
"""Rebuild synchronized native domain targets and run their real CTests.

Uses the existing SpaceHero CMake cache and changes only the exploration-test
toggle. Tests consume actual Autonomy libraries, generated messages and native
BT/transport. No staged replacements are supplied to the compiler or linker.
"""

import argparse
import base64
import hashlib
import json
from pathlib import Path
import subprocess
import sys


TEST_TARGETS = (
    'autonomy_scheduler_motion_test',
    'autonomy_shared_bspline_cache_test',
    'autonomy_reader_retirement_test', 'autonomy_native_observation_source_test',
    'autonomy_observation_ingress_test', 'autonomy_native_sensor_messages_test',
    'autonomy_native_vista_test', 'autonomy_vista_server_test',
    'autonomy_vista_incremental_graph_test',
    'autonomy_bt_runner_test', 'autonomy_typed_task_test', 'autonomy_navigation_task_test',
    'autonomy_result_response_wait_test', 'autonomy_path_reference_test',
    'autonomy_motion_dispatch_test', 'autonomy_motion_dispatch_gate_test',
    'autonomy_multisensor_exploration_test', 'autonomy_exploration_session_test',
    'autonomy_exploration_phase_test', 'autonomy_native_exploration_bt_test',
    'autonomy_navigation_dispatcher_test', 'autonomy_exploration_owner_test',
    'autonomy_hierarchical_coverage_strategy_test', 'autonomy_exploration_input_prefix_test',
    'autonomy_exploration_loss_recovery_test',
    'control.policy_differential_drive_test', 'control.policy_timed_trajectory_test',
    'control.policy_local_ground_test', 'control.policy_local_trajectory_safety_test',
    'control.policy_trajectory_tracker_test',
    'control.policy_native_spline_tracker_test',
)

REMOTE_PROGRAM = r'''
from pathlib import Path
import base64, hashlib, json, shlex, subprocess, tempfile

source_root = Path('/home/quandy/workspace/github/autonomy/src/autonomy')
native_root = '/workspace/autonomy/src/autonomy'
build = '/workspace/autonomy/build/autonomy'
log_root = Path(tempfile.mkdtemp(prefix='opt-native-exploration-v2.35-'))
expected = __EXPECTED__
protected_map = __MAP__
protected_task = __TASK__
test_targets = __TARGETS__
container = __CONTAINER__
jobs = __JOBS__

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None

def check_sources():
    mismatches = [n for n, value in expected.items() if digest(source_root / n) != value]
    if mismatches: raise RuntimeError('native source drift: ' + ', '.join(mismatches))
    current_map = {str(p.relative_to(source_root)): digest(p)
                   for p in sorted((source_root / 'autonomy/map').rglob('*')) if p.is_file()}
    if current_map != protected_map: raise RuntimeError('protected map changed')
    if any(digest(source_root / n) != value for n, value in protected_task.items()):
        raise RuntimeError('protected TaskServer changed')

def emit(event):
    print(json.dumps(event), flush=True)

def run(stage, inner, timeout):
    command = ['docker', 'exec', '-w', build, container, '/bin/bash', '-lc', shlex.join(inner)]
    log = log_root / (stage + '.log')
    emit({'event': 'start', 'stage': stage, 'command': command, 'remote_log': str(log)})
    try:
        with log.open('xb') as output:
            result = subprocess.run(command, stdout=output, stderr=subprocess.STDOUT, timeout=timeout)
        code = result.returncode
    except subprocess.TimeoutExpired:
        code = 124
        with log.open('ab') as output: output.write(b'\nNative validation command timed out.\n')
    emit({'event': 'step', 'stage': stage, 'command': command, 'returncode': code,
          'remote_log': str(log), 'log_base64': base64.b64encode(log.read_bytes()).decode()})
    return code == 0

try:
    check_sources()
    emit({'event': 'sources', 'source_sha256': expected, 'remote_logs': str(log_root)})
    passed = run('configure', ['cmake', '-S', native_root, '-B', build,
                              '-DAUTONOMY_BUILD_EXPLORATION_TESTS=ON'], 180)
    if passed:
        passed = run('build', ['cmake', '--build', build, '--target',
                              'autonomy_planning', 'autonomy_task', 'autonomy_control',
                              *test_targets, '-j', str(jobs)], 1200)
    if passed:
        pattern = ('^(autonomy[.](common[.]bspline_cache|native[.]vista|planning[.](vista_server|vista_incremental_graph)|'
                   'task[.](scheduler_motion|bt_runner|typed_task|navigation|result_response_wait|path_reference|'
                   'motion_dispatch|motion_dispatch_gate)|exploration[.].*)|'
                   'control[.]policy_(differential_drive|timed_trajectory|local_ground|local_trajectory_safety|trajectory_tracker|native_spline_tracker)_test)$')
        # Building a new target is insufficient if the CTest filter omits it.
        # Verify registration and selection before reporting successful tests.
        selection = subprocess.run(['docker', 'exec', '-w', build, container,
                                    'ctest', '--test-dir', build, '-R', pattern,
                                    '--show-only=json-v1'],
                                   capture_output=True, text=True, timeout=30, check=True)
        names = [item['name'] for item in json.loads(selection.stdout)['tests']]
        if len(names) != len(test_targets) or 'autonomy.task.scheduler_motion' not in names:
            raise RuntimeError('CTest selection does not cover every requested native test target: ' + repr(names))
        emit({'event': 'selection', 'selected_test_names': names,
              'selected_test_count': len(names), 'test_selection_verified': True})
        passed = run('tests', ['ctest', '--test-dir', build, '-R', pattern,
                              '--no-tests=error', '--output-on-failure'], 300)
    check_sources()
    emit({'event': 'finished', 'passed': passed, 'map_unchanged': True,
          'task_server_preserved': True})
except Exception as error:
    emit({'event': 'finished', 'passed': False, 'error': str(error)})
'''


def main():
    """Preserve individual native logs and the exact synchronized source inventory."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='quandy@192.168.66.124')
    parser.add_argument('--container', default='SpaceHero')
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--jobs', type=int, default=4)
    args = parser.parse_args()
    if args.report.exists(): parser.error('report already exists')
    if not 1 <= args.jobs <= 16 or args.host.startswith('-') or args.container.startswith('-'):
        parser.error('invalid host/container/jobs')
    manifest = json.loads(args.manifest.read_text())
    expected = manifest['complete_source_inventory']
    program = REMOTE_PROGRAM.replace('__EXPECTED__', repr(expected))
    program = program.replace('__MAP__', repr(manifest['protected_map_sha256']))
    program = program.replace('__TASK__', repr(manifest['protected_task_server_sha256']))
    program = program.replace('__TARGETS__', repr(TEST_TARGETS))
    program = program.replace('__CONTAINER__', repr(args.container)).replace('__JOBS__', repr(args.jobs))
    record = {'passed': False, 'steps': [], 'formal_native_rebuild_attempt': True,
              'scope': 'Explicit actual native domain/test targets. No full robot, trained model or physical stopping validation.'}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with tempfile_program(program) as program_file:
        process = subprocess.Popen(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
                                    args.host, 'python3', '-'], stdin=program_file,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for line in process.stdout:
            try: event = json.loads(line)
            except ValueError:
                record.setdefault('transport_output', []).append(line.rstrip()); continue
            kind = event.pop('event')
            if kind == 'step':
                data = base64.b64decode(event.pop('log_base64'))
                log = args.report.with_name(args.report.stem + '-' + event['stage'] + '.log')
                with log.open('xb') as output: output.write(data)
                event['log'] = str(log); event['log_sha256'] = hashlib.sha256(data).hexdigest()
                record['steps'].append(event)
                print(event['stage'], 'returncode', event['returncode'], 'log', log, flush=True)
            elif kind == 'start': print(event['stage'], 'started;', event['remote_log'], flush=True)
            else: record.update(event)
        record['ssh_returncode'] = process.wait()
    record['passed'] = record['passed'] and record['ssh_returncode'] == 0 and len(record['steps']) == 3
    record['formal_native_rebuild_verified'] = any(
        step['stage'] == 'build' and step['returncode'] == 0 for step in record['steps'])
    with args.report.open('x') as output:
        json.dump(record, output, indent=2); output.write('\n')
    print(json.dumps({'report': str(args.report), 'passed': record['passed'], 'error': record.get('error')}), flush=True)
    return 0 if record['passed'] else 1


def tempfile_program(program):
    """Give SSH stdin a file, so streaming stdout cannot deadlock a large payload."""
    import tempfile
    output = tempfile.TemporaryFile(mode='w+')
    output.write(program); output.seek(0)
    return output


if __name__ == '__main__':
    sys.exit(main())
