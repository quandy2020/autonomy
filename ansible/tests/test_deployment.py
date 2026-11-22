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
EVIDENCE = load('release_evidence', ROOT / 'scripts/release_evidence.py')
RETENTION = load('release_retention', ROOT / 'scripts/release_retention.py')
PACKAGE = load('package_install_tarball', ROOT.parent / 'tools/python/package_install_tarball.py')
with patch.dict('sys.modules', {'inspect_artifact': INSPECT}):
    RELEASE = load('prepare_release', ROOT / 'scripts/prepare_release.py')


class DeploymentTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name).resolve()

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
        cache = self.directory / 'cache'
        cache.mkdir()
        files = DELTA.inventory(base)
        first = DELTA.prepare_cached(archive, '/opt/autonomy', digest, files, cache)
        self.assertFalse(first['cache_hit'])
        with patch.object(DELTA, 'prepare', side_effect=AssertionError('repacked identical delta')):
            self.assertEqual(DELTA.prepare_cached(archive, '/opt/autonomy', digest, files, cache),
                             {**first, 'cache_hit': True})
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
        different = DELTA.prepare_cached(archive, '/opt/autonomy', digest, DELTA.inventory(base), cache)
        self.assertFalse(different['cache_hit'])
        self.assertNotEqual(different['payload_dir'], first['payload_dir'])
        stale = self.directory / 'stale'
        stale.mkdir()
        with self.assertRaisesRegex(ValueError, 'basis file changed'):
            DELTA.restore(base, stale, directory / 'reuse.json', plan['reuse_sha256'])

    def test_valid_artifact(self):
        result = INSPECT.inspect_artifact(self.archive(), '/opt/autonomy')
        self.assertEqual(result['architecture'], 'x86_64')
        self.assertEqual(len(result['sha256']), 64)

    def test_release_retention_protects_transitive_bases_and_respects_deployment_lock(self):
        root = self.directory / 'releases'
        active, base, ancestor, stale = ('a' * 64, 'b' * 64, 'c' * 64, 'd' * 64)
        for release, parent in [(active, base), (base, ancestor), (ancestor, None), (stale, None)]:
            manifest = root / release / 'share/autonomy/MANIFEST.json'
            manifest.parent.mkdir(parents=True)
            manifest.write_text(json.dumps({'deployment_base_sha256': parent}))
        prefix = self.directory / 'autonomy'
        prefix.symlink_to(root / active)
        report = RETENTION.retention(prefix, root, keep=0)
        self.assertEqual(report['cleanup_candidates'], [stale])
        self.assertIn('module base', report['protected'][ancestor])
        (root / '.deployment-lock').mkdir()
        self.assertEqual(RETENTION.retention(prefix, root, keep=0)['cleanup_candidates'], [])

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
                patch.object(PACKAGE.platform, 'machine', return_value='aarch64'), \
                patch.dict(os.environ, environment, clear=True):
            self.assertEqual(PACKAGE.main(['--skip-build', '--build-dir', str(build)]), 0)
        self.assertEqual(INSPECT.inspect_artifact(output, '/opt/autonomy')['sha256'],
                         output.with_name(output.name + '.sha256').read_text().split()[0])
        with tarfile.open(output) as archive:
            self.assertIn('autonomy/share/autonomy/payload', archive.getnames())
            metadata = json.load(archive.extractfile('autonomy/share/autonomy/MANIFEST.json'))
            self.assertEqual(metadata['cmake_build_type'], 'Debug')

    def test_release_packager_refuses_a_selected_build_tree(self):
        build = self.directory / 'modules'
        build.mkdir()
        (build / 'CMakeCache.txt').write_text('AUTONOMY_BUILD_MODULES:STRING=automsgs\n')
        output = self.directory / 'autonomy.tar.gz'
        with patch.object(PACKAGE, 'run') as run, patch('sys.stderr', new=io.StringIO()) as stderr:
            with self.assertRaises(SystemExit):
                PACKAGE.main(['--skip-build', '--build-dir', str(build), '--output', str(output)])
            run.assert_not_called()
            self.assertIn('requires a full build directory', stderr.getvalue())
        self.assertFalse(output.exists())

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
                self.assertEqual(AUDIT.main(['--environment', 'production', '--revision', 'd' * 40,
                                             '--deployment-revision', 'a' * 40,
                                             '--artifact-sha256', 'c' * 64, '--output', str(output)]), 0)
        record = json.loads(output.read_text())
        self.assertEqual(record['actor'], 'operator')
        self.assertEqual(record['revision'], 'd' * 40)
        self.assertEqual(record['deployment_revision'], 'a' * 40)
        self.assertEqual(record['extra_variables']['autonomy_expected_git_sha'], 'd' * 40)
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

    def test_bridge_probe_requires_an_application_heartbeat(self):
        bridge = self.directory / 'autonomy.bridge'
        for response, expected in [
            ({'status': {'code': 'STATUS_OK'}, 'sequence': 42, 'robot_time_ns': '123'}, 0),
            ({'status': {'code': 'STATUS_ERROR'}, 'sequence': 42, 'robot_time_ns': '123'}, 1),
            ({'status': {}, 'sequence': 41, 'robot_time_ns': '123'}, 1),
        ]:
            with self.subTest(response=response):
                bridge.write_text(f'#!/bin/sh\nprintf \'%s\\n\' \'{json.dumps(response)}\'\n')
                bridge.chmod(0o755)
                result = subprocess.run([
                    'python3', str(ROOT / 'scripts/probe_bridge.py'), '--bridge', str(bridge),
                    '--target', '127.0.0.1:5005',
                ], capture_output=True, text=True)
                self.assertEqual(result.returncode, expected, result.stdout + result.stderr)

    def dispatch(self, *arguments, environment=None):
        executable = self.directory / 'ansible-playbook'
        executable.write_text('''#!/usr/bin/env python3
import json, os, sys
if '--list-hosts' in sys.argv:
    print('hosts (0):' if os.environ.get('AUTONOMY_TEST_EMPTY') else 'hosts (1):')
if 'autonomy_play_mode=plan' in sys.argv and os.environ.get('AUTONOMY_TEST_PLAN_FAIL'):
    sys.exit(37)
for arg in sys.argv[1:]:
    if arg.startswith('autonomy_delta_cache_dir='):
        assert os.path.isdir(arg.split('=', 1)[1])
if os.environ.get('AUTONOMY_TEST_COLOR_ENV'):
    print(os.environ.get('ANSIBLE_FORCE_COLOR', '') + ':' + os.environ.get('ANSIBLE_NOCOLOR', ''), file=sys.stderr)
print(json.dumps(sys.argv[1:]))
if '--list-hosts' not in sys.argv:
    sys.exit(int(os.environ.get('AUTONOMY_TEST_DEPLOY_EXIT', '0')))
''')
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
        for mode, inventory in [('build', 'inventory/hosts.yml'), ('deploy', 'inventory/robots/hosts.yml'),
                                ('release', 'inventory/robots/hosts.yml'), ('cross-release', 'inventory/robots/hosts.yml')]:
            with self.subTest(default_inventory=mode):
                result = self.dispatch(mode, '--list-hosts', environment={'AUTONOMY_DEPLOY_ENV': ''})
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(json.loads(result.stdout.splitlines()[-1])[1], inventory)
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

    def test_maintenance_rejects_task_filters_that_could_change_transaction_scope(self):
        for mode in ('push', 'restart'):
            with self.subTest(mode=mode):
                result = self.dispatch(mode, 'robots', '--skip-tags=config')
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('Task selectors are not supported', result.stderr)

    def test_incremental_dispatch_removes_cache_and_preserves_ansible_status(self):
        for exit_code in (0, 42):
            with self.subTest(exit_code=exit_code):
                result = self.dispatch('deploy', 'robots', '-l', 'robot01', '--incremental', environment={
                    'AUTONOMY_TEST_DEPLOY_EXIT': str(exit_code), 'TMPDIR': str(self.directory),
                })
                self.assertEqual(result.returncode, exit_code, result.stderr)
                options = json.loads(result.stdout.splitlines()[-1])
                self.assertTrue(any(arg.startswith('autonomy_delta_cache_dir=') for arg in options))
                self.assertFalse(list(self.directory.glob('autonomy-delta.*')))

    def test_short_and_long_deploy_options_share_the_same_routing(self):
        base = self.directory / 'base package.tar.gz'
        base.touch()
        prefix = ('cross-release', 'robots', '-l', 'robot01', '-T', '15')
        short = self.dispatch(*prefix, '-R', 'custom config.yml', '-I', '-B', str(base),
                              '-m', 'autonomy/control', '--list-hosts')
        long = self.dispatch(*prefix, '--release-config=custom config.yml', '--incremental',
                             f'--base-artifact={base}', '--module=autonomy/control', '--list-hosts')
        self.assertEqual(short.returncode, 0, short.stderr)
        self.assertEqual(long.returncode, 0, long.stderr)
        self.assertEqual(short.stdout, long.stdout)
        self.assertIn('-T', json.loads(short.stdout.splitlines()[-1]))

        build = self.dispatch('build', 'staging', '-m', 'autonomy/control', '-S')
        self.assertEqual(build.returncode, 0, build.stderr)
        options = json.loads(build.stdout.splitlines()[-1])
        self.assertIn('autonomy_build_test=true', options)
        self.assertIn({'autonomy_build_modules': ['autonomy/control']},
                      [json.loads(arg) for arg in options if arg.startswith('{')])
        missing = self.dispatch(*prefix, '-R', '--list-hosts')
        self.assertNotEqual(missing.returncode, 0)
        self.assertIn('--release-config requires', missing.stderr)

    def test_color_modes_preserve_plain_logs_and_configure_ansible(self):
        args = ('deploy', 'robots', '--list-hosts')
        plain = self.dispatch(*args, environment={
            'AUTONOMY_LOG_COLOR': 'auto', 'ANSIBLE_FORCE_COLOR': '1', 'AUTONOMY_TEST_COLOR_ENV': '1',
        })
        self.assertEqual(plain.returncode, 0, plain.stderr)
        self.assertNotIn('\x1b[', plain.stdout)
        self.assertTrue(plain.stderr.startswith('0:'))

        forced = self.dispatch('-L', 'always', *args, environment={
            'NO_COLOR': '1', 'AUTONOMY_TEST_COLOR_ENV': '1',
        })
        self.assertEqual(forced.returncode, 0, forced.stderr)
        self.assertIn('\x1b[36m==> mode=', forced.stdout)
        self.assertEqual(forced.stderr.strip(), '1:0')
        self.assertEqual(json.loads(forced.stdout.splitlines()[-1]), json.loads(plain.stdout.splitlines()[-1]))

        muted = self.dispatch(*args, '--color=never', environment={
            'ANSIBLE_FORCE_COLOR': '1', 'AUTONOMY_TEST_COLOR_ENV': '1',
        })
        self.assertEqual(muted.returncode, 0, muted.stderr)
        self.assertNotIn('\x1b[', muted.stdout)
        self.assertEqual(muted.stderr.strip(), '0:1')

        checked = self.dispatch('deploy', 'robots', '-L', 'always', environment={'AUTONOMY_TEST_COLOR_ENV': '1'})
        self.assertEqual(checked.returncode, 0, checked.stderr)
        self.assertEqual(checked.stderr.splitlines(), ['0:1', '1:0'])

        rejected = self.dispatch('build', 'missing', '--color', 'always')
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn('\x1b[31mUnknown or non-canonical inventory', rejected.stderr)

        invalid = self.dispatch(*args, '--color=violet')
        self.assertNotEqual(invalid.returncode, 0)
        self.assertIn('--color requires auto, always or never', invalid.stderr)

    def test_help_colors_sections_and_includes_command_examples(self):
        colored = self.dispatch('--help', '--color=always')
        self.assertEqual(colored.returncode, 0, colored.stderr)
        self.assertIn('\x1b[36mExamples:\x1b[0m', colored.stdout)
        self.assertIn('\x1b[36mBuild and release options:\x1b[0m', colored.stdout)
        self.assertIn('\x1b[32m  ./deploy.sh build -K -S', colored.stdout)
        self.assertIn('cross-release robots -l robot01 -K -m autonomy/control', colored.stdout)
        self.assertIn('push robots -l robot01 -K -C -D', colored.stdout)
        self.assertIn('verify robots -l robot01 -e "autonomy_artifact_sha256=$sha"', colored.stdout)

        plain = self.dispatch('--help', '-L', 'never', environment={'AUTONOMY_LOG_COLOR': 'always'})
        self.assertEqual(plain.returncode, 0, plain.stderr)
        self.assertIn('Examples:', plain.stdout)
        self.assertNotIn('\x1b[', plain.stdout)

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
        self.assertTrue(commands[0][-2].endswith('/tools/python/clang_format_sources.py'))
        self.assertEqual(commands[0][-1], '--check')
        self.assertEqual(commands[1][5:7], ['colcon', 'build'])
        self.assertIn('--skip-build', commands[2])
        self.assertIn('--bundle-runtime', commands[2])
        self.assertEqual(commands[3][1], 'cp')
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
            if failing_step == 0:
                self.assertFalse(any('colcon' in command for command in calls))
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
        docker.write_text('#!/bin/sh\nif [ "$3" = uname ]; then echo x86_64; exit 0; fi\nexit 23\n')
        docker.chmod(0o755)
        result = self.dispatch('release', 'robots', '-l', 'robot01')
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('==> mode=', result.stdout)
        self.assertIn('Deployment was not started', result.stderr)

    def test_failed_preflight_stops_cross_release_before_build(self):
        result = self.dispatch('cross-release', 'robots', '-l', 'robot01',
                               environment={'AUTONOMY_TEST_PLAN_FAIL': '1'})
        self.assertEqual(result.returncode, 37, result.stderr)
        self.assertIn('Preflight selected hosts before building', result.stdout)
        self.assertNotIn('==> mode=', result.stdout)

        result = self.dispatch('deploy', 'staging', environment={'AUTONOMY_TEST_EMPTY': '1'})
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('matched no deploy targets', result.stderr)

    def test_module_selection_reaches_native_build_and_cannot_replace_a_release(self):
        result = self.dispatch('build', 'staging', '--module', 'autonomy/control', '--module=autolink', '--test')
        self.assertEqual(result.returncode, 0, result.stderr)
        arguments = json.loads(result.stdout.splitlines()[-1])
        self.assertIn({'autonomy_build_modules': ['autonomy/control', 'autolink']},
                      [json.loads(arg) for arg in arguments if arg.startswith('{')])
        self.assertIn('autonomy_build_test=true', arguments)
        result = self.dispatch('cross-release', 'robots', '-l', 'robot01', '--module', 'automsgs')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('requires both --module and --base-artifact', result.stderr)

    def test_cross_module_preparation_builds_without_packaging(self):
        artifact_dir = self.directory / 'artifacts'
        config = {'cross_profile': 'rockchip-rk3506-debian12', 'artifact_dir': str(artifact_dir)}
        with patch.object(RELEASE, 'run') as execute:
            result = RELEASE.prepare_cross(config, incremental=True, modules=['autonomy/control', 'automsgs'])
        self.assertEqual(result['modules'], ['automsgs', 'autonomy/control'])
        self.assertEqual(execute.call_args.args[2], 'build')
        self.assertIn('--incremental', execute.call_args.args)
        self.assertNotIn('autonomy_artifact_path', result)
        self.assertFalse(artifact_dir.exists())

    def test_module_archive_keeps_base_runtime_and_configuration(self):
        profile = 'rockchip-rk3506-debian12'
        config = {'cross_profile': profile, 'artifact_dir': str(self.directory / 'artifacts')}
        identity = {'sysroot_sha256': 'a' * 64, 'image_id': 'sha256:' + 'b' * 64}
        staged = self.directory / 'base/autonomy'
        for name, value in {'bin/task': b'unchanged task', 'lib/libautomsgs.so': b'old',
                            'share/autonomy/config/task/state': b'base config',
                            'share/unchanged-resource': os.urandom(32 * 1024)}.items():
            file = staged / name
            file.parent.mkdir(parents=True, exist_ok=True)
            file.write_bytes(value)
        base = self.directory / 'base.tar.gz'
        metadata = {'schema_version': 1, 'install_prefix': '/opt/autonomy', 'architecture': 'armv7l',
                    'cross_profile': profile, 'target_os': 'debian', 'target_version': '12',
                    'target_dpkg_arch': 'armhf', 'target_abi': 'arm-linux-gnueabihf', **identity}
        base_sha = PACKAGE.write_archive(staged, base, metadata, reproducible=True)

        def build(*command):
            self.assertEqual(command[2], 'install')
            self.assertIn('--incremental', command)
            at = command.index('--expected-builder')
            self.assertEqual(command[at + 1:at + 3], (identity['sysroot_sha256'], identity['image_id']))
            stage = Path(command[command.index('--output-dir') + 1])
            installed = stage / 'install/opt/autonomy'
            for name, value in {'lib/libautomsgs.so': b'new',
                                'share/autonomy/config/task/state': b'ignored config',
                                'lib/cmake/automsgs/automsgs-config.cmake': b'ignored export'}.items():
                file = installed / name
                file.parent.mkdir(parents=True, exist_ok=True)
                file.write_bytes(value)
            (stage / 'build-provenance.json').write_text(json.dumps(identity))

        with patch.object(RELEASE, 'run', side_effect=build):
            result = RELEASE.prepare_cross(config, modules=['automsgs'], base_artifact=base)
        archive = Path(result['autonomy_artifact_path'])
        self.assertEqual(INSPECT.inspect_artifact(archive, '/opt/autonomy')['deployment_base_sha256'], base_sha)
        with tarfile.open(archive) as content:
            self.assertEqual(content.extractfile('autonomy/bin/task').read(), b'unchanged task')
            self.assertEqual(content.extractfile('autonomy/lib/libautomsgs.so').read(), b'new')
            self.assertEqual(content.extractfile('autonomy/share/autonomy/config/task/state').read(), b'base config')
            self.assertNotIn('autonomy/lib/cmake/automsgs/automsgs-config.cmake', content.getnames())
        transfer = self.directory / 'transfer'
        transfer.mkdir()
        plan = DELTA.prepare(archive, '/opt/autonomy', result['autonomy_artifact_sha256'],
                             DELTA.inventory(staged), transfer)
        self.assertTrue(plan['use_delta'])
        with tarfile.open(transfer / 'delta.tar.gz') as content:
            self.assertIn('autonomy/lib/libautomsgs.so', content.getnames())
            self.assertNotIn('autonomy/share/unchanged-resource', content.getnames())

    def test_module_archive_removes_paths_owned_by_the_previous_module_update(self):
        profile = 'rockchip-rk3506-debian12'
        identity = {'sysroot_sha256': 'a' * 64, 'image_id': 'sha256:' + 'b' * 64}
        staged = self.directory / 'base/autonomy'
        for name, value in {'lib/libautomsgs-old.so': b'old', 'bin/task': b'base task'}.items():
            file = staged / name
            file.parent.mkdir(parents=True, exist_ok=True)
            file.write_bytes(value)
        base = self.directory / 'base.tar.gz'
        metadata = {'schema_version': 1, 'install_prefix': '/opt/autonomy', 'architecture': 'armv7l',
                    'cross_profile': profile, 'target_os': 'debian', 'target_version': '12',
                    'target_dpkg_arch': 'armhf', 'target_abi': 'arm-linux-gnueabihf',
                    'modules': ['automsgs'],
                    'module_runtime_files': ['lib/libautomsgs-old.so'], **identity}
        base_sha = PACKAGE.write_archive(staged, base, metadata, reproducible=True)
        update = self.directory / 'update.tar.gz'
        update_root = self.directory / 'update/autonomy'
        (update_root / 'lib').mkdir(parents=True)
        (update_root / 'lib/libautomsgs-new.so').write_bytes(b'new')
        PACKAGE.write_archive(update_root, update, metadata, reproducible=True)
        merged = self.directory / 'merged.tar.gz'
        RELEASE.merge_module_archive(base, update, merged, '/opt/autonomy', base_sha,
                                     ['automsgs'], ['lib/libautomsgs-new.so'])
        with tarfile.open(merged) as content:
            self.assertNotIn('autonomy/lib/libautomsgs-old.so', content.getnames())
            self.assertEqual(content.extractfile('autonomy/lib/libautomsgs-new.so').read(), b'new')

    def test_module_archive_rejects_unsafe_runtime_file_paths(self):
        base = self.directory / 'base.tar.gz'
        update = self.directory / 'update.tar.gz'
        root = self.directory / 'root/autonomy'
        (root / 'lib').mkdir(parents=True)
        (root / 'lib/libautomsgs.so').write_bytes(b'new')
        metadata = {'install_prefix': '/opt/autonomy', 'architecture': 'armv7l'}
        base_sha = PACKAGE.write_archive(root, base, metadata, reproducible=True)
        PACKAGE.write_archive(root, update, metadata, reproducible=True)

        with self.assertRaisesRegex(ValueError, 'module runtime file list is invalid'):
            RELEASE.merge_module_archive(base, update, self.directory / 'merged.tar.gz',
                                         '/opt/autonomy', base_sha, ['automsgs'], ['../../escape'])

    def test_promotion_requires_verified_staging_for_the_exact_release(self):
        revision, digest, profile = '1' * 40, 'a' * 64, 'nvidia-orin-nx-jetpack6'
        audit = {'environment': 'staging', 'revision': revision, 'artifact_sha256': digest,
                 'cross_profile': profile, 'inventory_tree': 'inventory/staging',
                 'host_limit': 'robot01', 'host_listing': 'hosts (1):\n  robot01'}
        accepted = EVIDENCE.record(audit)
        EVIDENCE.verify(accepted, revision, digest, profile)
        for mismatch in [('2' * 40, digest, profile), (revision, 'b' * 64, profile),
                         (revision, digest, 'nvidia-orin-nano-jetpack6')]:
            with self.assertRaisesRegex(ValueError, 'does not match'):
                EVIDENCE.verify(accepted, *mismatch)
        with self.assertRaisesRegex(ValueError, 'completed and verified'):
            EVIDENCE.verify(audit, revision, digest, profile)
        with self.assertRaisesRegex(ValueError, 'at least one target'):
            EVIDENCE.record({**audit, 'host_listing': 'hosts (0):'})

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
