"""Enforce collection boundaries and the public deployment policy."""
from pathlib import Path
import json
import re
import shutil
import subprocess
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]


class ContractTest(unittest.TestCase):
    def test_every_role_has_an_entrypoint_contract_and_explicit_dependencies(self):
        for role in (ROOT / 'roles').iterdir():
            if not role.is_dir():
                continue
            with self.subTest(role=role.name):
                self.assertRegex(role.name, r'^[a-z][a-z0-9_]*$')
                self.assertTrue((role / 'tasks/main.yml').is_file())
                metadata = yaml.safe_load((role / 'meta/main.yml').read_text())
                self.assertIn('dependencies', metadata)
                contract = (role / 'README.md').read_text()
                for field in ['输入：', '输出：', '依赖：']:
                    self.assertIn(field, contract)
                self.assertTrue((ROOT / 'molecule/roles' / role.name / 'converge.yml').is_file())

    def test_role_sources_and_scenarios_are_not_hidden_by_gitignore(self):
        paths = list((ROOT / 'roles').glob('*/tasks/main.yml'))
        paths += list((ROOT / 'molecule/roles').glob('*/converge.yml'))
        result = subprocess.run(['git', 'check-ignore', '--', *map(str, paths)],
                                cwd=ROOT, text=True, capture_output=True)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)

    def test_production_playbooks_only_orchestrate_fully_qualified_roles(self):
        for path in (ROOT / 'playbooks').glob('*.yml'):
            for play in yaml.safe_load(path.read_text()):
                self.assertNotIn('tasks', play, path)
                self.assertNotIn('post_tasks', play, path)
                for role in play.get('roles', []):
                    self.assertTrue(role['role'].startswith('openbot.autonomy_ops.'), path)
                    self.assertIn(role['role'].split('.')[-1], role['tags'], path)
        for path in (ROOT / 'roles').rglob('*.yml'):
            self.assertNotIn('include_role:', path.read_text(), path)

    def test_collection_dependencies_and_controller_versions_are_exact(self):
        requirements = yaml.safe_load((ROOT / 'requirements.yml').read_text())
        collection = yaml.safe_load((ROOT / 'galaxy.yml').read_text())
        for dependency in requirements['collections']:
            self.assertRegex(dependency['version'], r'^\d+\.\d+\.\d+$')
            self.assertEqual(collection['dependencies'][dependency['name']], dependency['version'])
        for path in [ROOT / 'requirements-ci.txt', ROOT / 'requirements-dev.txt']:
            for line in path.read_text().splitlines():
                if line and not line.startswith(('#', '-r ', '-c ')):
                    self.assertIsNotNone(re.fullmatch(r'[a-zA-Z0-9_\[\]-]+==\d+\.\d+\.\d+', line))

    def test_local_collection_contains_runtime_support_and_excludes_controller_data(self):
        manifest = ROOT / '.ansible/collections/ansible_collections/openbot/autonomy_ops/FILES.json'
        if not manifest.exists():
            self.skipTest('install the local collection first')
        names = {entry['name'] for entry in json.loads(manifest.read_text())['files']}
        self.assertIn('scripts/inspect_artifact.py', names)
        self.assertIn('vars/lock.yml', names)
        self.assertFalse(any(name.startswith(('inventory/', 'tests/', 'molecule/')) for name in names))
        self.assertNotIn('.vault_pass', names)
        self.assertNotIn('scripts/deployment_audit.py', names)
        self.assertNotIn('scripts/prepare_release.py', names)
        self.assertNotIn('scripts/controller_env.sh', names)
        self.assertNotIn('vars/release.yml', names)
        self.assertNotIn('vars/release-demo.yml', names)

    @unittest.skipUnless(shutil.which('ansible-playbook'), 'Ansible tooling is required')
    def test_deployment_policy_rejects_invalid_build_and_production_settings(self):
        base = ['ansible-playbook', '-i', 'localhost,', 'tests/version_lock.yml']
        for arguments, expected in [([], 0),
                                    (['-e', 'autonomy_play_mode=build', '-e', 'autonomy_repo_version=main'], 2),
                                    (['-e', 'autonomy_profile=production', '-e', 'autonomy_play_mode=full',
                                      '-e', 'autonomy_deploy_mode=artifact'], 2),
                                    (['-e', 'autonomy_profile=production', '-e', 'autonomy_system_install=false'], 2),
                                    (['-e', 'autonomy_system_install=false', '-e', 'autonomy_install_dependencies=true'], 2)]:
            result = subprocess.run(base + arguments, cwd=ROOT, capture_output=True, text=True)
            self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
