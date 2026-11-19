#!/usr/bin/env python3
"""Start, inspect and stop the managed AutoSim complete-coverage demo stack."""
from pathlib import Path
import argparse
import json
import os
import signal
import subprocess
import time


class DemoRuntime:
    """Manage only the exact recorded processes of one coverage demonstration."""

    def __init__(self):
        self.coverage = Path(__file__).resolve().parent.parent
        self.source = self.coverage.parents[3]
        self.workspace = self.source.parent.parent
        self.logs = self.workspace / 'artifacts/coverage-runtime'
        self.logs.mkdir(parents=True, exist_ok=True)
        self.record = self.logs / 'processes.json'
        self.processes = json.loads(self.record.read_text()) if self.record.exists() else {}
        binary = self.workspace / 'build/autonomy/bin'
        self.commands = {
            'sim': ['/opt/venv/bin/python3', '-m', 'autosim', '--config', str(self.source / 'autosim/config/coverage.yaml')],
            'planning': [str(binary / 'autonomy.planning'), '--conf=' + str(self.source / 'autonomy/planning/conf/planner.pb.txt')],
            'control': [str(binary / 'autonomy.control'), '--conf=' + str(self.coverage / 'conf/coverage_controller.pb.txt')],
            'task': [str(binary / 'autonomy.task'), '--enable_coverage', '--coverage_config', str(self.coverage / 'conf/autosim_coverage.json')],
            'autoviz': [str(binary / 'autoviz')],
        }

    def live(self, item):
        try:
            proc = Path('/proc') / str(item['pid'])
            if (proc / 'stat').read_text().split()[2] == 'Z':
                return False
            current = (proc / 'cmdline').read_bytes().rstrip(b'\0').split(b'\0')
            return [v.decode() for v in current] == item['command']
        except (FileNotFoundError, PermissionError):
            return False

    def start(self, headless=False):
        env = os.environ.copy()
        env['AUTONOMY_PATH'] = str(self.source)
        env['AUTOLINK_PATH'] = str(self.source / 'autolink')
        env['DISPLAY'] = env.get('DISPLAY', ':0')
        env['LD_LIBRARY_PATH'] = str(self.workspace / 'build/autonomy/lib') + ':/usr/local/lib:' + env.get('LD_LIBRARY_PATH', '')
        env['PYTHONPATH'] = ':'.join([str(self.workspace / 'build/autonomy/python'), str(self.workspace / 'build/autonomy/automsgs/proto/gen/python'), str(self.source / 'autosim'), env.get('PYTHONPATH', '')])
        for name, command in self.commands.items():
            if headless and name == 'autoviz':
                continue
            if name in self.processes and self.live(self.processes[name]):
                continue
            with (self.logs / (name + '.log')).open('w') as output:
                process = subprocess.Popen(command, cwd=self.workspace, env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            self.processes[name] = {'pid': process.pid, 'command': command}
        self.record.write_text(json.dumps(self.processes, indent=2) + '\n')
        self.status()

    def stop(self):
        for name in ['autoviz', 'task', 'control', 'planning', 'sim']:
            item = self.processes.get(name)
            if not item or not self.live(item):
                continue
            os.kill(item['pid'], signal.SIGTERM)
            deadline = time.monotonic() + 5
            while self.live(item) and time.monotonic() < deadline:
                time.sleep(.05)
            if self.live(item):
                raise RuntimeError(f'{name} is still retiring; its dependent processes remain running')
        self.status()

    def status(self):
        for name, item in self.processes.items():
            print(f'{name}: {"running" if self.live(item) else "stopped"} pid={item["pid"]}')
        print('Logs:', self.logs)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['start', 'stop', 'status'])
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()
    runtime = DemoRuntime()
    if args.action == 'start':
        runtime.start(args.headless)
    else:
        getattr(runtime, args.action)()
