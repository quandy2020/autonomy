"""Run the real activation and health recovery roles using local, pre-staged artifacts."""
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which('ansible-playbook') and shutil.which('timeout'), 'requires local Ansible and timeout')
class RolloutTransactionTest(unittest.TestCase):
    def test_failed_probe_restores_verified_release_and_config(self):
        with tempfile.TemporaryDirectory(prefix='autonomy-rollout-test-') as directory:
            home = Path(directory).resolve()
            prefix = home / 'opt/autonomy'
            config = home / 'etc/autonomy'
            release_dir = home / 'opt/releases'
            inventory = home / 'inventory.json'
            inventory.write_text(json.dumps({'autonomy': {'hosts': {'localhost': {
                'ansible_connection': 'local', 'ansible_python_interpreter': sys.executable,
                'ansible_become': False,
            }}}}))
            (home / 'etc').mkdir()
            release_dir.mkdir(parents=True)

            def stage(value, base_sha=None):
                artifact = home / (value + '.tar.gz')
                manifest = {'architecture': 'aarch64', 'install_prefix': str(prefix)}
                if base_sha:
                    manifest.update(schema_version=1, cross_profile='rockchip-rk3588-ubuntu2204',
                                    target_dpkg_arch='arm64', target_os='ubuntu', target_version='22.04',
                                    target_abi='aarch64-linux-gnu', sysroot_sha256='1' * 64,
                                    image_id='sha256:' + '2' * 64, deployment_base_sha256=base_sha,
                                    modules=['automsgs'], git_sha='f' * 40)
                files = {'share/autonomy/MANIFEST.json': json.dumps(manifest).encode(),
                         'share/autonomy/config/task/state.txt': value.encode()}
                if base_sha:
                    files['share/module-state'] = value.encode()
                with tarfile.open(artifact, 'w:gz') as tar:
                    for name, content in files.items():
                        info = tarfile.TarInfo('autonomy/' + name)
                        info.size = len(content)
                        tar.addfile(info, io.BytesIO(content))
                checksum = hashlib.sha256(artifact.read_bytes()).hexdigest()
                # This fixture isolates activation; archive extraction already has its own Linux test.
                for name, content in files.items():
                    path = release_dir / checksum / name
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_bytes(content)
                return artifact, checksum

            old, old_sha = stage('healthy')
            new, new_sha = stage('unhealthy')
            dpkg = home / 'dpkg'
            dpkg.write_text('#!/bin/sh\necho arm64\n')
            dpkg.chmod(0o755)
            baseline = {
                'autonomy_deploy_mode': 'artifact', 'autonomy_system_install': False,
                'autonomy_install_dependencies': False, 'autonomy_controller_root': str(ROOT),
                'autonomy_install_prefix': str(prefix), 'autonomy_config_dir': str(config),
                'autonomy_release_dir': str(release_dir), 'autonomy_data_dir': str(home / 'data'),
                'autonomy_services': [], 'autonomy_config_components': ['task'],
                'ansible_architecture': 'aarch64',
                'autonomy_require_health_probe': True, 'autonomy_run_verify': False,
                'ansible_distribution': 'Ubuntu', 'ansible_distribution_version': '22.04',
                'autonomy_cross_profile': 'rockchip-rk3588-ubuntu2204',
                'autonomy_health_probe_argv': [
                    sys.executable, '-c',
                    'from pathlib import Path; import sys; state = Path(sys.argv[2]); '
                    'sys.exit(Path(sys.argv[1]).read_text() not in ("healthy", "site config") '
                    'or (state.is_file() and state.read_text() == "module-bad"))',
                    str(config / 'task/state.txt'), str(prefix / 'share/module-state'),
                ],
            }

            def invoke(mode, archive, sha, check=False, **overrides):
                values = {**baseline, 'autonomy_play_mode': mode, 'autonomy_artifact_path': str(archive),
                          'autonomy_artifact_sha256': sha, **overrides}
                tags = {'verify': 'services', 'plan': 'preflight', 'push': 'config',
                        'restart': 'restart'}.get(mode, 'artifact')
                return subprocess.run(
                    ['ansible-playbook', '-i', str(inventory), 'playbooks/site.yml',
                     '--tags', tags, '-e', json.dumps(values), *(['--check', '--diff'] if check else [])],
                    cwd=ROOT, env={**os.environ, 'ANSIBLE_NOCOLOR': '1',
                                    'PATH': str(home) + os.pathsep + os.environ['PATH']},
                    capture_output=True, text=True,
                )

            def ok(result):
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

            ok(invoke('full', old, old_sha))
            self.assertEqual(prefix.resolve(), release_dir / old_sha)
            self.assertEqual(json.loads((release_dir / '.last-verified').read_text())['release'],
                             str(release_dir / old_sha))
            repeated = invoke('full', old, old_sha)
            ok(repeated)
            self.assertIn('changed=0', repeated.stdout)

            # The running release can have a different, healthy site configuration.
            (config / 'task/state.txt').write_text('site config')
            failed_retry = invoke('full', old, old_sha, autonomy_health_probe_argv=[
                sys.executable, '-c',
                'from pathlib import Path; import sys; '
                'sys.exit(Path(sys.argv[1]).read_text() != "site config")',
                str(config / 'task/state.txt'),
            ])
            self.assertNotEqual(failed_retry.returncode, 0, failed_retry.stdout)
            self.assertEqual((config / 'task/state.txt').read_text(), 'site config')
            self.assertIn('Previous verified release restored and checked', failed_retry.stdout)
            self.assertEqual(prefix.resolve(), release_dir / old_sha)
            self.assertFalse((release_dir / '.deployment-lock').exists())
            (config / 'task/state.txt').write_text('healthy')

            failed = invoke('full', new, new_sha)
            self.assertNotEqual(failed.returncode, 0, failed.stdout)
            self.assertIn('Previous verified release restored and checked', failed.stdout)
            self.assertEqual(prefix.resolve(), release_dir / old_sha)
            self.assertEqual((config / 'task/state.txt').read_text(), 'healthy')
            self.assertFalse((release_dir / '.deployment-lock').exists())
            ok(invoke('verify', old, old_sha))
            busy = release_dir / '.deployment-lock'
            busy.mkdir()
            reject = invoke('full', new, new_sha)
            self.assertNotEqual(reject.returncode, 0, reject.stdout)
            self.assertTrue(busy.is_dir())
            self.assertEqual(prefix.resolve(), release_dir / old_sha)

            busy.rmdir()
            (config / 'task/state.txt').write_text('site config')
            wrong, wrong_sha = stage('module-wrong', '0' * 64)
            rejected = invoke('full', wrong, wrong_sha)
            self.assertNotEqual(rejected.returncode, 0, rejected.stdout)
            self.assertIn('Module deployment requires the active base archive', rejected.stdout)
            self.assertEqual(prefix.resolve(), release_dir / old_sha)

            bad, bad_sha = stage('module-bad', old_sha)
            failed = invoke('full', bad, bad_sha)
            self.assertNotEqual(failed.returncode, 0, failed.stdout)
            self.assertIn('Previous verified release restored and checked', failed.stdout)
            self.assertEqual(prefix.resolve(), release_dir / old_sha)
            self.assertEqual((config / 'task/state.txt').read_text(), 'site config')

            good, good_sha = stage('module-good', old_sha)
            ok(invoke('full', good, good_sha))
            self.assertEqual(prefix.resolve(), release_dir / good_sha)
            self.assertEqual((config / 'task/state.txt').read_text(), 'site config')
            repeated = invoke('full', good, good_sha)
            ok(repeated)
            self.assertIn('changed=0', repeated.stdout)

            contract = {'autonomy_min_free_disk_mb': 1, 'autonomy_plan_module_base_sha256': old_sha,
                        'autonomy_plan_module_names': ['automsgs'], 'autonomy_plan_module_git_sha': 'f' * 40,
                        'autonomy_cross_release_profile': 'rockchip-rk3588-ubuntu2204'}
            ok(invoke('plan', good, good_sha, **contract))
            contract['autonomy_plan_module_git_sha'] = 'e' * 40
            rejected_plan = invoke('plan', good, good_sha, **contract)
            self.assertNotEqual(rejected_plan.returncode, 0)
            self.assertIn('Stopped before compiling', rejected_plan.stdout)

            settings = home / 'source/autonomy/task/conf'
            settings.mkdir(parents=True)
            (settings / 'state.txt').write_text('healthy')
            values = {'autonomy_config_source_dir': str(home / 'source')}
            ok(invoke('push', good, good_sha, check=True, **values))
            self.assertEqual((config / 'task/state.txt').read_text(), 'site config')
            self.assertFalse(list(config.parent.glob('.autonomy-config-*')))
            ok(invoke('push', good, good_sha, **values))
            self.assertEqual((config / 'task/state.txt').read_text(), 'healthy')
            snapshots = list(config.parent.glob('.autonomy-config-*'))
            self.assertEqual(len(snapshots), 1)
            self.assertEqual((snapshots[0] / 'config/task/state.txt').read_text(), 'site config')

            (settings / 'state.txt').write_text('unhealthy')
            busy.mkdir()
            owner = busy / 'owner.json'
            owner.write_text('{"mode":"full"}')
            for mode in ('push', 'restart'):
                blocked = invoke(mode, good, good_sha, **values)
                self.assertNotEqual(blocked.returncode, 0, blocked.stdout)
                self.assertEqual(owner.read_text(), '{"mode":"full"}')
                self.assertEqual((config / 'task/state.txt').read_text(), 'healthy')
            ok(invoke('push', good, good_sha, check=True, **values))
            self.assertTrue(owner.is_file())
            self.assertEqual((config / 'task/state.txt').read_text(), 'healthy')
            owner.unlink()
            busy.rmdir()

            failed_push = invoke('push', good, good_sha, **values)
            self.assertNotEqual(failed_push.returncode, 0, failed_push.stdout)
            self.assertIn('Previous configuration restored and checked', failed_push.stdout)
            self.assertEqual((config / 'task/state.txt').read_text(), 'healthy')
            self.assertEqual(prefix.resolve(), release_dir / good_sha)
            self.assertFalse(busy.exists())
            failed_configs = list(config.parent.glob('.autonomy-config-*/failed-config/task/state.txt'))
            self.assertEqual([path.read_text() for path in failed_configs], ['unhealthy'])
