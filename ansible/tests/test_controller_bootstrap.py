"""Exercise first-run setup without network access or changing the real venv."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ControllerBootstrapTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / 'scripts').mkdir()
        shutil.copyfile(ROOT / 'scripts/controller_env.sh', self.root / 'scripts/controller_env.sh')
        for name in ['requirements-ci.txt', 'requirements-lock.txt', 'requirements.yml']:
            (self.root / name).write_text('fixture\n')
        tools = self.root / 'tools'
        tools.mkdir()
        python = tools / 'python3.12'
        python.write_text(f'#!{sys.executable}\n' + '''import json, os, pathlib, sys
if sys.argv[1:] == ['-m', 'pip', '--version']:
    sys.exit(1 if os.environ.get('BOOTSTRAP_NO_PIP') and '.venv' in sys.argv[0] else 0)
with open(os.environ['BOOTSTRAP_CALLS'], 'a') as log:
    log.write(json.dumps(sys.argv[1:]) + '\\n')
if sys.argv[1:3] == ['-m', 'venv']:
    tools = pathlib.Path(sys.argv[-1]) / 'bin'
    tools.mkdir(parents=True)
    (tools / 'python3').symlink_to(pathlib.Path(__file__).resolve())
    for name in ['ansible-playbook', 'ansible-galaxy']:
        script = tools / name
        script.write_text('#!/bin/sh\\nexit 0\\n')
        script.chmod(0o755)
elif os.environ.get('BOOTSTRAP_FAIL'):
    sys.exit(19)
''')
        python.chmod(0o755)
        self.calls = self.root / 'calls.jsonl'
        self.environment = {**os.environ, 'PATH': str(tools) + os.pathsep + os.environ['PATH'],
                            'BOOTSTRAP_CALLS': str(self.calls)}

    def setup_controller(self, **environment):
        return subprocess.run(['bash', '-c', 'source "$1"; command -v ansible-playbook',
                               'bootstrap', str(self.root / 'scripts/controller_env.sh')],
                              env={**self.environment, **environment}, text=True, capture_output=True)

    def test_first_run_reuse_and_requirement_changes(self):
        result = self.setup_controller()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), str(self.root / '.venv/bin/ansible-playbook'))
        calls = self.calls.read_text().splitlines()
        self.assertEqual(len(calls), 2)  # venv creation and pinned pip installation
        self.assertEqual(self.setup_controller().returncode, 0)
        self.assertEqual(self.calls.read_text().splitlines(), calls)
        (self.root / 'requirements-lock.txt').write_text('updated fixture\n')
        self.assertEqual(self.setup_controller().returncode, 0)
        self.assertEqual(len(self.calls.read_text().splitlines()), 3)

    def test_failed_install_does_not_mark_setup_complete(self):
        result = self.setup_controller(BOOTSTRAP_FAIL='1')
        self.assertEqual(result.returncode, 19)
        self.assertFalse((self.root / '.venv/.deployment-requirements.sha256').exists())

    def test_host_pip_bootstraps_venv_without_ensurepip(self):
        result = self.setup_controller(BOOTSTRAP_NO_PIP='1')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--python', self.calls.read_text())


if __name__ == '__main__':
    unittest.main()
