"""Optional Docker integration: run with AUTONOMY_TEST_CONTAINER set."""
import hashlib
import io
import json
import os
from pathlib import Path
import platform
import subprocess
import tarfile
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
CONTAINER = os.environ.get('AUTONOMY_TEST_CONTAINER')


@unittest.skipUnless(CONTAINER, 'set AUTONOMY_TEST_CONTAINER for isolated Docker integration')
class ArtifactLifecycleTest(unittest.TestCase):
    def test_activation_idempotence_and_failed_upgrade(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            inventory = directory / 'inventory.json'
            inventory.write_text(json.dumps({'all': {'hosts': {CONTAINER: {
                'ansible_connection': 'community.docker.docker',
                'ansible_python_interpreter': '/usr/bin/python3',
            }}}}))
            prefix = '/tmp/autonomy-artifact-' + directory.name
            base = {'autonomy_install_prefix': prefix, 'autonomy_services': ['task'],
                    'autonomy_deploy_mode': 'artifact', 'autonomy_user': 'root',
                    'autonomy_config_dir': prefix + '-config', 'autonomy_config_components': ['task']}
            unchanged = os.urandom(64 * 1024)

            def docker(*arguments):
                return subprocess.check_output(['docker', 'exec', CONTAINER, *arguments], text=True).strip()

            def archive(version, binary=True):
                path = directory / f'{version}.tar.gz'
                manifest = {'install_prefix': prefix, 'architecture': platform.machine(), 'git_sha': version}
                entries = {'share/autonomy/MANIFEST.json': json.dumps(manifest),
                           'share/autonomy/config/task/navigator.pb.txt': version,
                           'share/autonomy/unchanged.bin': unchanged}
                if binary:
                    entries['bin/autonomy.task'] = '#!/bin/sh\nexit 0\n'
                if version == 'first':
                    entries['old-only'] = 'must not survive the upgrade'
                with tarfile.open(path, 'w:gz') as output:
                    for name, content in entries.items():
                        info = tarfile.TarInfo(Path(prefix).name + '/' + name)
                        data = content if isinstance(content, bytes) else content.encode()
                        info.size = len(data)
                        info.mode = 0o755 if name.startswith('bin/') else (0o640 if '/config/' in name else 0o644)
                        output.addfile(info, io.BytesIO(data))
                return path

            def deploy(path, **overrides):
                variables = {**base, 'autonomy_artifact_path': str(path), **overrides}
                return subprocess.run(['ansible-playbook', '-i', str(inventory), 'tests/artifact.yml',
                                       '-e', json.dumps(variables)], cwd=ROOT, text=True, capture_output=True)

            first = archive('first')
            result = deploy(first)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            previous = docker('readlink', '-f', prefix)
            result = deploy(first)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn('changed=0', result.stdout)
            second = archive('second')
            result = deploy(second, autonomy_artifact_incremental=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn('Reused ', result.stdout)
            active = docker('readlink', '-f', prefix)
            self.assertTrue(active.endswith(hashlib.sha256(second.read_bytes()).hexdigest()))
            self.assertEqual(docker('readlink', '-f', prefix + '.previous'), previous)
            self.assertEqual(docker('test', '!', '-e', prefix + '/old-only'), '')
            for path, overrides in [(second, {'autonomy_artifact_sha256': 'invalid'}),
                                    (archive('missing-binary', binary=False), {})]:
                result = deploy(path, **overrides)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(docker('readlink', '-f', prefix), active)
            self.assertEqual(docker('find', prefix + '-releases', '-name', '.stage-*'), '')
            self.assertEqual(docker('cat', prefix + '-config/task/navigator.pb.txt'), 'second')
            source = directory / 'source/autonomy/task/conf'
            source.mkdir(parents=True)
            (source / 'navigator.pb.txt').write_text('pushed-from-controller')
            result = deploy(second, autonomy_play_mode='push', autonomy_config_source_dir=str(directory / 'source'))
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(docker('cat', prefix + '-config/task/navigator.pb.txt'), 'pushed-from-controller')
