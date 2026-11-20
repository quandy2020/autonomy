"""Offline regressions for deployment dispatch, archives and packaging."""
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
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
DELTA = load('artifact_delta', ROOT / 'scripts/artifact_delta.py')
AUDIT = load('deployment_audit', ROOT / 'scripts/deployment_audit.py')
PACKAGE = load('package_install_tarball', ROOT.parent / 'tools/python/package_install_tarball.py')
with patch.dict('sys.modules', {'inspect_artifact': INSPECT}):
    RELEASE = load('prepare_release', ROOT / 'scripts/prepare_release.py')


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

    def test_delta_reuses_only_verified_files_and_keeps_removed_paths_out(self):
        import hashlib

        base = self.directory / 'old' / 'autonomy'
        base.mkdir(parents=True)
        unchanged = os.urandom(32 * 1024)
        (base / 'same').write_bytes(unchanged)
        (base / 'removed').write_text('old')
        (base / 'updated').write_bytes(b'old')
        archive = self.directory / 'new.tar.gz'
        with tarfile.open(archive, 'w:gz') as output:
            for name, data in {
                'same': unchanged,
                'updated': b'new',
                'share/autonomy/MANIFEST.json': json.dumps(
                    {'architecture': 'x86_64', 'install_prefix': '/opt/autonomy'}).encode(),
            }.items():
                entry = tarfile.TarInfo('autonomy/' + name)
                entry.size = len(data)
                output.addfile(entry, io.BytesIO(data))

        directory = self.directory / 'delta'
        directory.mkdir()
        digest = hashlib.sha256(archive.read_bytes()).hexdigest()
        with self.assertRaisesRegex(ValueError, 'artifact changed'):
            DELTA.prepare(archive, '/opt/autonomy', '0' * 64, DELTA.inventory(base), directory)
        plan = DELTA.prepare(archive, '/opt/autonomy', digest, DELTA.inventory(base), directory)
        self.assertTrue(plan['use_delta'])
        self.assertEqual(plan['reused'], 1)
        stage = self.directory / 'stage'
        stage.mkdir()
        DELTA.restore(base, stage, directory / 'reuse.json', plan['reuse_sha256'])
        with tarfile.open(directory / 'delta.tar.gz') as payload:
            self.assertNotIn('autonomy/same', payload.getnames())
            payload.extractall(stage, filter='data')
        self.assertEqual((stage / 'autonomy/same').read_bytes(), unchanged)
        self.assertEqual((stage / 'autonomy/updated').read_bytes(), b'new')
        self.assertFalse((stage / 'autonomy/removed').exists())
        self.assertFalse((stage / 'autonomy/same').samefile(base / 'same'))

        (base / 'same').write_bytes(b'changed after inventory')
        stale = self.directory / 'stale'
        stale.mkdir()
        with self.assertRaisesRegex(ValueError, 'basis file changed'):
            DELTA.restore(base, stale, directory / 'reuse.json', plan['reuse_sha256'])

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

    def test_runtime_bundle_preserves_application_libraries_and_excludes_platform_runtime(self):
        prefix = self.directory / 'install'
        (prefix / 'bin').mkdir(parents=True)
        (prefix / 'bin/app').write_bytes(b'\x7fELFfixture')
        (prefix / 'lib').mkdir()
        (prefix / 'lib/libown.so').write_bytes(b'installed-own-library')
        sdk = self.directory / 'sdk with spaces'
        sdk.mkdir()
        (sdk / 'libfixture.so.1').write_bytes(b'third-party-library')
        output = (f'libfixture.so.1 => {sdk}/libfixture.so.1 (0x1234)\n'
                  'libown.so => /build/libown.so (0x5678)\n'
                  'libc.so.6 => /lib/libc.so.6 (0x1111)\n'
                  'libstdc++.so.6 => /lib/libstdc++.so.6 (0x2222)\n')
        with patch.object(PACKAGE.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, output, '')):
            libraries = PACKAGE.bundle_runtime_libraries(self.directory / 'build', prefix)
        self.assertEqual(libraries, ['libfixture.so.1'])
        self.assertEqual((prefix / 'lib/libfixture.so.1').read_bytes(), b'third-party-library')
        self.assertEqual((prefix / 'lib/libown.so').read_bytes(), b'installed-own-library')
        self.assertFalse((prefix / 'lib/libc.so.6').exists())

        with patch.object(PACKAGE.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, 'libmissing.so => not found', '')):
            with self.assertRaises(ValueError):
                PACKAGE.bundle_runtime_libraries(self.directory / 'build', prefix)

    def test_autoviz_bundle_includes_matching_qt_plugins_and_their_dependencies(self):
        prefix = self.directory / 'install'
        (prefix / 'bin').mkdir(parents=True)
        (prefix / 'bin/autoviz').write_bytes(b'\x7fELFfixture')
        plugins = self.directory / 'qt/plugins'
        (plugins / 'platforms').mkdir(parents=True)
        (plugins / 'platforms/libqxcb.so').write_bytes(b'\x7fELFplugin')
        dependency = self.directory / 'libxcb-fixture.so.1'
        dependency.write_bytes(b'linked-dependency')

        def query(command, **kwargs):
            output = str(plugins) if command[0] == 'qmake6' else f'libxcb-fixture.so.1 => {dependency} (0x1234)\n'
            return subprocess.CompletedProcess(command, 0, output, '')

        with patch.object(PACKAGE.subprocess, 'run', side_effect=query) as runner:
            PACKAGE.bundle_runtime_libraries(self.directory / 'build', prefix)
        plugin = prefix / 'lib/qt6/plugins/platforms/libqxcb.so'
        self.assertEqual(plugin.read_bytes(), b'\x7fELFplugin')
        self.assertTrue(any(call.args[0] == ['ldd', str(plugin)] for call in runner.call_args_list))
        self.assertEqual((prefix / 'lib/libxcb-fixture.so.1').read_bytes(), b'linked-dependency')
        self.assertIn('Plugins=lib/qt6/plugins', (prefix / 'bin/qt.conf').read_text())

    def test_library_deduplication_preserves_aliases_distinct_versions_and_archive_hardlinks(self):
        prefix = self.directory / 'autonomy'
        library = prefix / 'lib'
        (library / 'OGRE').mkdir(parents=True)
        payload = b'\x7fELFidentical-library'
        first = library / 'OGRE/Plugin.so.1'
        duplicate = library / 'Plugin.so.1'
        first.write_bytes(payload)
        duplicate.write_bytes(payload)
        distinct = library / 'Plugin.so.2'
        distinct.write_bytes(payload[:-1] + b'X')
        alias = library / 'Plugin.so'
        alias.symlink_to('Plugin.so.1')
        with patch.object(PACKAGE.os, 'link', side_effect=OSError('link failed')):
            with self.assertRaises(OSError):
                PACKAGE.deduplicate_runtime_libraries(library)
        self.assertEqual(duplicate.read_bytes(), payload)
        self.assertFalse(first.samefile(duplicate))
        self.assertEqual(PACKAGE.deduplicate_runtime_libraries(library), len(payload))
        self.assertTrue(first.samefile(duplicate))
        self.assertTrue(alias.is_symlink())
        self.assertEqual(alias.readlink(), Path('Plugin.so.1'))
        self.assertFalse(first.samefile(distinct))
        self.assertEqual(PACKAGE.deduplicate_runtime_libraries(library), 0)
        manifest = prefix / 'share/autonomy/MANIFEST.json'
        manifest.parent.mkdir(parents=True)
        manifest.write_text(json.dumps({'install_prefix': '/opt/autonomy', 'architecture': 'x86_64'}))
        archive = self.directory / 'deduplicated.tar.gz'
        with tarfile.open(archive, 'w:gz') as tar:
            tar.add(prefix, arcname='autonomy')
        INSPECT.inspect_artifact(archive, '/opt/autonomy')
        extracted = self.directory / 'extracted'
        with tarfile.open(archive) as tar:
            self.assertTrue(any(member.islnk() for member in tar.getmembers()))
            tar.extractall(extracted, filter='data')
        self.assertTrue((extracted / 'autonomy/lib/OGRE/Plugin.so.1').samefile(extracted / 'autonomy/lib/Plugin.so.1'))

    def test_package_existing_build_without_reconfiguration(self):
        build = self.directory / 'build'
        project = self.directory / 'project'
        project.mkdir()
        (project / 'payload').write_text('fixture')
        (project / 'CMakeLists.txt').write_text(
            'cmake_minimum_required(VERSION 3.20)\nproject(fixture NONE)\n'
            'install(FILES payload DESTINATION share/autonomy)\n')
        subprocess.run(['cmake', '-S', str(project), '-B', str(build), '-DCMAKE_BUILD_TYPE=Debug'], check=True, capture_output=True)
        output = self.directory / '.autonomy/artifacts/autonomy-fixture.tar.gz'
        environment = {key: value for key, value in os.environ.items() if key != 'OUTPUT'}
        environment['HOME'] = str(self.directory)
        with patch.object(PACKAGE, 'bundle_configuration'), \
                patch.object(PACKAGE, 'git_describe', return_value='fixture'), \
                patch.dict(os.environ, environment, clear=True):
            self.assertEqual(PACKAGE.main(['--skip-build', '--build-dir', str(build)]), 0)
        self.assertEqual(INSPECT.inspect_artifact(output, '/opt/autonomy')['sha256'],
                         output.with_name(output.name + '.sha256').read_text().split()[0])
        with tarfile.open(output) as archive:
            self.assertIn('autonomy/share/autonomy/payload', archive.getnames())
            metadata = json.load(archive.extractfile('autonomy/share/autonomy/MANIFEST.json'))
            self.assertEqual(metadata['cmake_build_type'], 'Debug')

    @unittest.skipUnless(shutil.which('patchelf') and shutil.which('cc') and shutil.which('c++'),
                         'ELF relocation regression requires patchelf and C/C++ compilers')
    def test_real_elf_resolves_indirect_libraries_and_resources_after_relocation(self):
        prefix = self.directory / 'install with spaces'
        for directory in ('bin', 'lib', 'share/autolink/conf'):
            (prefix / directory).mkdir(parents=True)
        (prefix / 'share/autolink/conf/autolink.pb.conf').touch()
        include = self.directory / 'include/autolink/conf'
        include.mkdir(parents=True)
        (include / 'conf.hpp').write_text(
            'namespace autolink { namespace conf {\n'
            'constexpr auto kDefaultDistributionHome = "/unusable-build-tree";\n'
            'constexpr auto kDefaultConfDir = "/unusable-build-tree/conf";\n'
            'constexpr auto kDefaultPluginIndexDir = "/unusable-build-tree/plugins";\n'
            '}}\n')
        leaf = self.directory / 'leaf.c'
        middle = self.directory / 'middle.c'
        app = self.directory / 'app.cpp'
        leaf.write_text('int leaf(void) { return 42; }\n')
        middle.write_text('int leaf(void); int middle(void) { return leaf(); }\n')
        app.write_text('#include <iostream>\n#include "autolink/common/environment.hpp"\n'
                       'extern "C" int middle(void);\n'
                       'int main(int argc, char** argv) {\n'
                       'if (argc != 2 || middle() != 42) return 1;\n'
                       'std::cout << autolink::common::DistributionHome() << "|"\n'
                       '<< autolink::common::WorkRoot() << "|" << argv[1]; }\n')
        for command in (
            ['cc', '-shared', '-fPIC', str(leaf), '-o', str(prefix / 'lib/libleaf.so')],
            ['cc', '-shared', '-fPIC', str(middle), '-L' + str(prefix / 'lib'), '-lleaf',
             '-o', str(prefix / 'lib/libmiddle.so')],
            ['c++', '-std=c++17', str(app), '-I' + str(ROOT.parent / 'autolink'),
             '-I' + str(self.directory / 'include'), '-L' + str(prefix / 'lib'), '-lmiddle',
             '-Wl,-rpath-link,' + str(prefix / 'lib'), '-o', str(prefix / 'bin/app')],
        ):
            subprocess.run(command, check=True, capture_output=True)
        PACKAGE.configure_runtime_rpaths(prefix)
        moved = prefix.rename(self.directory / 'relocated with spaces')
        self.assertEqual((moved / 'bin/app').read_bytes()[:4], b'\x7fELF')
        self.assertFalse(list((moved / 'bin').glob('.*.elf')))
        env = {key: value for key, value in os.environ.items()
               if key not in ('LD_LIBRARY_PATH', 'AUTOLINK_PATH', 'AUTONOMY_PATH', 'AUTOLINK_DISTRIBUTION_HOME')}
        result = subprocess.run([str(moved / 'bin/app'), 'one argument'], cwd=self.directory,
                                env=env, check=True, capture_output=True, text=True)
        self.assertEqual(result.stdout, f'{moved}|{moved}/share/autolink|one argument')
        (moved / 'share/autolink/conf/autolink.pb.conf').unlink()
        result = subprocess.run([str(moved / 'bin/app'), 'one argument'], cwd=self.directory,
                                env=env, check=True, capture_output=True, text=True)
        self.assertEqual(result.stdout, '/unusable-build-tree|/unusable-build-tree|one argument')

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
        result = self.dispatch('deploy', 'robots', '--incremental', '--list-hosts')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('autonomy_artifact_incremental=true', json.loads(result.stdout.splitlines()[-1]))

    def release_config(self):
        return {'container': 'fixture-builder', 'workspace': '/workspace with spaces',
                'build_dir': '/workspace with spaces/build/autonomy',
                'build_args': ['--packages-up-to', 'autonomy'],
                'artifact_path': str(self.directory / 'output with spaces/autonomy.tar.gz'),
                'install_prefix': '/opt/autonomy'}

    def test_release_publishes_validated_artifact_and_returns_its_identity(self):
        config = self.release_config()
        expected_output = Path(config['artifact_path'])
        config['artifact_path'] = '~/output with spaces/autonomy.tar.gz'
        config['bundle_runtime'] = True
        archive = self.archive()

        def execute(arguments, **kwargs):
            if arguments[1] == 'cp':
                shutil.copyfile(archive, arguments[-1])
            return subprocess.CompletedProcess(arguments, 0)

        with patch.object(RELEASE.subprocess, 'run', side_effect=execute) as calls, \
                patch.dict(os.environ, {'HOME': str(self.directory)}):
            variables = RELEASE.prepare(config)
        output = Path(variables['autonomy_artifact_path'])
        self.assertEqual(output, expected_output)
        self.assertEqual(output.read_bytes(), archive.read_bytes())
        self.assertEqual(output.with_name(output.name + '.sha256').read_text(),
                         variables['autonomy_artifact_sha256'] + '  autonomy.tar.gz\n')
        commands = [call.args[0] for call in calls.call_args_list]
        self.assertEqual(commands[0][3], config['workspace'])
        self.assertEqual(commands[0][5:7], ['colcon', 'build'])
        self.assertIn('--skip-build', commands[1])
        self.assertIn('--bundle-runtime', commands[1])
        self.assertEqual(commands[2][1], 'cp')
        self.assertEqual(commands[-1][3:6], ['rm', '-rf', '--'])

    def test_failed_release_never_replaces_existing_artifact(self):
        config = self.release_config()
        output = Path(config['artifact_path'])
        output.parent.mkdir()
        output.write_bytes(b'previous-release')
        for failing_step in [0, 1]:
            calls = []

            def execute(arguments, **kwargs):
                calls.append(arguments)
                if len(calls) - 1 == failing_step:
                    raise subprocess.CalledProcessError(1, arguments)
                return subprocess.CompletedProcess(arguments, 0)

            with self.subTest(step=failing_step), patch.object(RELEASE.subprocess, 'run', side_effect=execute):
                with self.assertRaises(subprocess.CalledProcessError):
                    RELEASE.prepare(config)
            self.assertFalse(any(command[1] == 'cp' for command in calls))
            self.assertEqual(output.read_bytes(), b'previous-release')
            self.assertEqual(calls[-1][3], 'rm')

        def corrupt_archive(arguments, **kwargs):
            if arguments[1] == 'cp':
                Path(arguments[-1]).write_bytes(b'corrupt')
            return subprocess.CompletedProcess(arguments, 0)

        with patch.object(RELEASE.subprocess, 'run', side_effect=corrupt_archive):
            with self.assertRaises(tarfile.ReadError):
                RELEASE.prepare(config)
        self.assertEqual(output.read_bytes(), b'previous-release')

    def test_release_rejects_invalid_configuration_before_running_commands(self):
        for overrides in [{'container': '--privileged'}, {'workspace': '../source'},
                          {'install_prefix': '/'}, {'build_args': 'shell command'}, {'unknown': True},
                          {'artifact_path': str(self.directory / 'source.cpp')}, {'bundle_runtime': 'true'}]:
            with self.subTest(overrides=overrides), patch.object(RELEASE.subprocess, 'run') as execute:
                with self.assertRaises(ValueError):
                    RELEASE.prepare({**self.release_config(), **overrides})
                execute.assert_not_called()

    def test_release_dispatch_preserves_options_and_skips_build_for_inspection(self):
        result = self.dispatch('release', 'robots', '--release-config', '/missing.yml',
                               '-l', 'robot01', '--list-hosts')
        self.assertEqual(result.returncode, 0, result.stderr)
        arguments = json.loads(result.stdout.splitlines()[-1])
        self.assertIn('autonomy_deploy_mode=artifact', arguments)
        self.assertIn('autonomy_install_dependencies=false', arguments)
        self.assertEqual(arguments[-3:], ['-l', 'robot01', '--list-hosts'])
        self.assertNotIn('--release-config', arguments)
        self.assertNotEqual(self.dispatch('release', 'production').returncode, 0)

    def test_release_dispatch_stops_before_deployment_on_build_failure(self):
        docker = self.directory / 'docker'
        docker.write_text('#!/bin/sh\nexit 23\n')
        docker.chmod(0o755)
        result = self.dispatch('release', 'robots', '-l', 'robot01')
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('==> mode=', result.stdout)
        self.assertIn('Deployment was not started', result.stderr)

    def test_cross_release_stages_and_labels_real_target_artifact(self):
        config = {'cross_profile': 'rockchip-rk3506-debian12',
                  'artifact_dir': str(self.directory / 'artifacts')}

        def install(*command, machine=40):
            directory = Path(command[command.index('--output-dir') + 1])
            executable = directory / 'install/opt/autonomy/bin/autonomy.task'
            executable.parent.mkdir(parents=True)
            elf = bytearray(52)
            elf[:6] = b'\x7fELF\x01\x01'
            elf[16:18] = b'\x02\x00'
            elf[18:20] = machine.to_bytes(2, 'little')
            elf[36:40] = b'\x00\x04\x00\x05'
            executable.write_bytes(elf)
            executable.chmod(0o755)

        with patch.object(RELEASE, 'run', side_effect=lambda *args: install(*args, machine=62)):
            with self.assertRaisesRegex(ValueError, 'wrong ELF machine'):
                RELEASE.prepare_cross(config)
        self.assertEqual(list((self.directory / 'artifacts').iterdir()), [])

        with patch.object(RELEASE, 'run', side_effect=install) as build:
            variables = RELEASE.prepare_cross(config, incremental=True)
        self.assertIn('--incremental', build.call_args.args)
        archive = Path(variables['autonomy_artifact_path'])
        metadata = INSPECT.inspect_artifact(archive, '/opt/autonomy')
        self.assertEqual(metadata['architecture'], 'armv7l')
        self.assertEqual(metadata['cross_profile'], config['cross_profile'])
        self.assertEqual(metadata['target_dpkg_arch'], 'armhf')
        self.assertEqual(variables['autonomy_artifact_sha256'], metadata['sha256'])
        self.assertIn(metadata['sha256'], archive.name)
        self.assertEqual(list(archive.parent.glob('.autonomy-cross-*')), [])

    def test_repacked_unchanged_cross_install_keeps_the_same_release_identity(self):
        manifest = {'architecture': 'aarch64', 'install_prefix': '/opt/autonomy'}
        first = self.directory / 'first.tar.gz'
        second = self.directory / 'second.tar.gz'
        for index, archive in enumerate((first, second)):
            staged = self.directory / str(index) / 'autonomy'
            staged.mkdir(parents=True)
            executable = staged / 'payload'
            executable.write_text('identical files')
            os.utime(executable, (index, index))
            PACKAGE.write_archive(staged, archive, manifest, reproducible=True)
        self.assertEqual(first.read_bytes(), second.read_bytes())

    def test_cross_release_refuses_production_and_skips_build_for_host_listing(self):
        result = self.dispatch('cross-release', 'robots', '--release-config', '/missing.yml',
                               '-l', 'robot01', '--list-hosts')
        self.assertEqual(result.returncode, 0, result.stderr)
        arguments = json.loads(result.stdout.splitlines()[-1])
        self.assertIn('autonomy_deploy_mode=artifact', arguments)
        self.assertEqual(arguments[-3:], ['-l', 'robot01', '--list-hosts'])
        self.assertNotEqual(self.dispatch('cross-release', 'production').returncode, 0)
        self.assertNotEqual(self.dispatch('cross-release', 'robots').returncode, 0)


if __name__ == '__main__':
    unittest.main()
