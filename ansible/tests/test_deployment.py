"""Offline regressions for deployment dispatch, archives and packaging."""
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import tarfile
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


INSPECT = load('inspect_artifact', ROOT / 'scripts/inspect_artifact.py')
AUDIT = load('deployment_audit', ROOT / 'scripts/deployment_audit.py')
PACKAGE = load('package_install_tarball', ROOT.parent / 'tools/python/package_install_tarball.py')


class DeploymentTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)

    def archive(self, extra=None, manifest=None):
        path = self.directory / 'artifact.tar.gz'
        metadata = manifest or {'install_prefix': '/opt/autonomy', 'architecture': 'x86_64'}
        with tarfile.open(path, 'w:gz') as tar:
            data = json.dumps(metadata).encode()
            info = tarfile.TarInfo('autonomy/share/autonomy/MANIFEST.json')
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
            if extra is not None:
                tar.addfile(extra)
        return path

    def test_valid_artifact(self):
        result = INSPECT.inspect_artifact(self.archive(), '/opt/autonomy')
        self.assertEqual(result['architecture'], 'x86_64')
        self.assertEqual(len(result['sha256']), 64)

    def test_archive_escape_and_special_files_are_rejected(self):
        for name, kind, link in [
            ('../escape', tarfile.REGTYPE, ''),
            ('/absolute', tarfile.REGTYPE, ''),
            ('autonomy/lib/link', tarfile.SYMTYPE, '../../../escape'),
            ('autonomy/link', tarfile.LNKTYPE, '/etc/passwd'),
            ('autonomy/device', tarfile.CHRTYPE, ''),
        ]:
            with self.subTest(name=name):
                entry = tarfile.TarInfo(name)
                entry.type, entry.linkname = kind, link
                with self.assertRaises(ValueError):
                    INSPECT.inspect_artifact(self.archive(entry), '/opt/autonomy')

    def test_wrong_prefix_and_missing_architecture_are_rejected(self):
        for metadata in [{'install_prefix': '/other', 'architecture': 'x86_64'},
                         {'install_prefix': '/opt/autonomy'}]:
            with self.assertRaises(ValueError):
                INSPECT.inspect_artifact(self.archive(manifest=metadata), '/opt/autonomy')

    def test_package_root_and_component_config_layout(self):
        self.assertEqual(PACKAGE.repository_root(), ROOT.parent)
        source = self.directory / 'repo/autonomy/task/conf'
        source.mkdir(parents=True)
        (source / 'navigator.pb.txt').write_text('fixture')
        prefix = self.directory / 'stage'
        PACKAGE.bundle_configuration(self.directory / 'repo', prefix)
        self.assertEqual((prefix / 'share/autonomy/config/task/navigator.pb.txt').read_text(), 'fixture')

    def test_package_existing_build_without_reconfiguration(self):
        build = self.directory / 'build'
        project = self.directory / 'project'
        project.mkdir()
        (project / 'payload').write_text('fixture')
        (project / 'CMakeLists.txt').write_text(
            'cmake_minimum_required(VERSION 3.20)\nproject(fixture NONE)\n'
            'install(FILES payload DESTINATION share/autonomy)\n')
        subprocess.run(['cmake', '-S', str(project), '-B', str(build), '-DCMAKE_BUILD_TYPE=Debug'], check=True, capture_output=True)
        output = self.directory / 'autonomy.tar.gz'
        with patch.object(PACKAGE, 'bundle_configuration'):
            self.assertEqual(PACKAGE.main(['--skip-build', '--build-dir', str(build), '--output', str(output)]), 0)
        self.assertEqual(INSPECT.inspect_artifact(output, '/opt/autonomy')['sha256'],
                         output.with_name(output.name + '.sha256').read_text().split()[0])
        with tarfile.open(output) as archive:
            self.assertIn('autonomy/share/autonomy/payload', archive.getnames())
            metadata = json.load(archive.extractfile('autonomy/share/autonomy/MANIFEST.json'))
            self.assertEqual(metadata['cmake_build_type'], 'Debug')

    def test_audit_records_only_public_release_context(self):
        output = self.directory / 'audit.json'
        with patch.object(AUDIT.subprocess, 'check_output', side_effect=['a' * 40, 'b' * 40, 'hosts (1): robot01']):
            with patch.dict(os.environ, {'GITHUB_ACTOR': 'operator', 'UNRELATED_TOKEN': 'dummy-private-value'}):
                self.assertEqual(AUDIT.main(['--environment', 'production', '--revision', 'a' * 40,
                                             '--artifact-sha256', 'c' * 64, '--output', str(output)]), 0)
        record = json.loads(output.read_text())
        self.assertEqual(record['actor'], 'operator')
        self.assertEqual(record['inventory_tree'], 'b' * 40)
        self.assertNotIn('dummy-private-value', output.read_text())

    def test_audit_rejects_an_empty_host_selection(self):
        output = self.directory / 'empty-audit.json'
        with patch.object(AUDIT.subprocess, 'check_output', side_effect=['a' * 40, 'b' * 40, 'hosts (0):']):
            with patch('sys.stderr', new=io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    AUDIT.main(['--environment', 'production', '--revision', 'a' * 40,
                                '--artifact-sha256', 'c' * 64, '--output', str(output)])
        self.assertEqual(error.exception.code, 2)
        self.assertFalse(output.exists())

    def dispatch(self, *arguments, environment=None):
        executable = self.directory / 'ansible-playbook'
        executable.write_text('#!/usr/bin/env python3\nimport json, sys\nprint(json.dumps(sys.argv[1:]))\n')
        executable.chmod(0o755)
        galaxy = self.directory / 'ansible-galaxy'
        galaxy.write_text('#!/bin/sh\nexit 0\n')
        galaxy.chmod(0o755)
        env = os.environ.copy()
        env.pop('AUTONOMY_DEPLOY_ENV', None)
        env['PATH'] = str(self.directory) + os.pathsep + env['PATH']
        env.update(environment or {})
        return subprocess.run(['bash', str(ROOT / 'deploy.sh'), *arguments],
                              text=True, capture_output=True, env=env, stdin=subprocess.DEVNULL)

    def test_dispatch_modes_and_explicit_inventory_override(self):
        for mode, deployment in [('build', 'build'), ('deploy', 'artifact')]:
            with self.subTest(mode=mode):
                result = self.dispatch(mode, 'staging', environment={'AUTONOMY_DEPLOY_ENV': 'robots'})
                self.assertEqual(result.returncode, 0, result.stderr)
                arguments = json.loads(result.stdout.splitlines()[-1])
                self.assertEqual(arguments[1], 'inventory/staging/hosts.yml')
                self.assertIn('playbooks/staging.yml', arguments)
                self.assertIn(f'autonomy_deploy_mode={deployment}', arguments)
                self.assertNotIn('--ask-become-pass', arguments)
        self.assertNotEqual(self.dispatch('build', 'unknown').returncode, 0)
        self.assertEqual(self.dispatch('--help').returncode, 0)


if __name__ == '__main__':
    unittest.main()
