"""Offline regression tests. Execution is deferred at the user's request."""
import argparse
import copy
import json
import os
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import cross_build as cb


class CrossBuildTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name).resolve()
        self.meta = cb.load_profile('rockchip-rk3506-debian12')

    def file(self, name, data=b''):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        return path

    def elf(self, name, machine=40, bits=1, flags=0x5000400):
        data = bytearray(64)
        data[:6] = b'\x7fELF' + bytes([bits, 1])
        struct.pack_into('<H', data, 18, machine)
        struct.pack_into('<I', data, 36 if bits == 1 else 48, flags)
        return self.file(name, data)

    def sysroot(self):
        self.file('usr/include/stdio.h')
        self.file('usr/lib/arm-linux-gnueabihf/crt1.o')
        self.elf('usr/lib/arm-linux-gnueabihf/libc.so.6')
        self.elf('usr/lib/arm-linux-gnueabihf/libstdc++.so.6')
        self.elf('lib/ld-linux-armhf.so.3')
        self.file('etc/os-release', b'ID=debian\nVERSION_ID="12"\n')

    def test_profiles_have_unique_valid_names(self):
        names = [p.name for p in cb.profile_dirs()]
        self.assertEqual(len(names), 5)
        for name in names:
            self.assertEqual(cb.load_profile(name)['platform'], name)

    def test_reject_path_traversal_and_metadata_name_mismatch(self):
        with self.assertRaises(ValueError):
            cb.load_profile('../other')
        meta = copy.deepcopy(self.meta)
        meta['model'] = 'other'
        with self.assertRaises(ValueError):
            cb.validate_profile(meta, self.meta['platform'])

    def test_absolute_links_stay_in_sysroot_and_merged_usr_works(self):
        self.file('usr/lib/libtest.so')
        (self.root / 'lib').symlink_to('/usr/lib')
        cb.normalize_links(self.root)
        self.assertEqual((self.root / 'lib/libtest.so').resolve(), self.root / 'usr/lib/libtest.so')
        self.assertEqual(os.readlink(self.root / 'lib'), 'usr/lib')

    def test_reject_relative_escape_even_if_destination_does_not_exist(self):
        (self.root / 'bad').symlink_to('../outside')
        with self.assertRaises(ValueError):
            cb.normalize_links(self.root)

    def test_reject_wrong_architecture_and_soft_float(self):
        self.sysroot()
        cb.verify_tree(self.root, self.meta)
        self.elf('usr/lib/arm-linux-gnueabihf/libc.so.6', machine=183, bits=2)
        with self.assertRaises(ValueError):
            cb.verify_tree(self.root, self.meta)
        self.elf('usr/lib/arm-linux-gnueabihf/libc.so.6', flags=0x5000200)
        with self.assertRaises(ValueError):
            cb.verify_tree(self.root, self.meta)

    def test_reject_wrong_distribution(self):
        self.sysroot()
        self.file('etc/os-release', b'ID=ubuntu\nVERSION_ID="22.04"\n')
        with self.assertRaises(ValueError):
            cb.verify_tree(self.root, self.meta)

    def test_failed_replacement_restores_previous_tree(self):
        old = self.root / 'sysroot'
        old.mkdir()
        (old / 'old').touch()
        new = self.root / 'staging'
        new.mkdir()
        real_replace = os.replace
        def fail_publish(src, dst):
            if Path(src) == new:
                raise OSError('publish failed')
            return real_replace(src, dst)
        with patch.object(cb.os, 'replace', side_effect=fail_publish):
            with self.assertRaises(OSError):
                cb.publish_tree(new, old)
        self.assertTrue((old / 'old').exists())
        self.assertTrue(new.exists())

    def test_lock_rejects_overlapping_operations(self):
        with cb.profile_lock(self.root, self.meta['platform']):
            with self.assertRaises(ValueError):
                with cb.profile_lock(self.root, self.meta['platform']):
                    pass

    def test_build_keeps_cmake_arguments_separate_and_mounts_source_readonly(self):
        workspace = self.root
        self.root = workspace / 'work/sysroots' / self.meta['platform']
        self.sysroot()
        source = workspace / 'source with spaces'
        source.mkdir()
        (source / 'CMakeLists.txt').touch()
        args = argparse.Namespace(source=str(source), image=None,
                                  docker_platform='linux/amd64', command='build',
                                  output_dir=None, install_prefix=None, incremental=False,
                                  cmake_args=['--', '-DMESSAGE=a b;$(touch nope)'])
        with patch.object(cb, 'WORK', workspace / 'work'), patch.object(cb, 'run') as execute:
            cb.build_project(args, self.meta)
        command = execute.call_args.args[0]
        self.assertEqual(command[:2], ['docker', 'run'])
        self.assertIn('autonomy.platform.x86_64', command)
        self.assertEqual(command[-1], '-DMESSAGE=a b;$(touch nope)')
        self.assertIn(f'type=bind,src={source},dst=/src,readonly', command)
        self.assertIn(f'type=bind,src={self.root},dst=/sysroot,readonly', command)

    def test_cache_reuses_objects_across_staging_paths_but_isolates_sdk_and_image(self):
        base = self.root
        self.root = base / 'work/sysroots' / self.meta['platform']
        self.sysroot()
        source = base / 'src'
        source.mkdir()
        (source / 'CMakeLists.txt').touch()
        args = argparse.Namespace(docker_platform='linux/amd64', install_prefix='/opt/autonomy',
                                  source=str(source), image=None, command='install',
                                  cmake_args=[], output_dir=str(base / 'stage-one'), incremental=True)
        with patch.object(cb, 'WORK', base / 'work'), \
                patch.object(cb, 'local_image_id', return_value='sha256:one'), \
                patch.object(cb, 'source_date_epoch', return_value='0'), patch.object(cb, 'run') as execute:
            key = cb.incremental_build_dir(args, self.meta, source, self.root, 'sha256:one', [])
            cb.build_project(args, self.meta)
            first = execute.call_args.args[0]
            args.output_dir = str(base / 'stage-two')
            cb.build_project(args, self.meta)
            second = execute.call_args.args[0]
            for command in (first, second):
                self.assertIn(f'type=bind,src={key},dst=/build', command)
                self.assertIn('sha256:one', command)
            self.assertIn(f'type=bind,src={base}/stage-one/install,dst=/install', first)
            self.assertIn(f'type=bind,src={base}/stage-two/install,dst=/install', second)
            self.file('.cross-build.json', b'new collection timestamp')
            self.assertEqual(key, cb.incremental_build_dir(
                args, self.meta, source, self.root, 'sha256:one', [],
            ))
            self.assertNotEqual(key, cb.incremental_build_dir(
                args, self.meta, source, self.root, 'sha256:two', [],
            ))
            self.assertNotEqual(key, cb.incremental_build_dir(
                args, self.meta, base / 'other-source', self.root, 'sha256:one', [],
            ))
            self.assertNotEqual(key, cb.incremental_build_dir(
                args, self.meta, source, self.root, 'sha256:one', ['-DBUILD_TEST=ON'],
            ))
            self.file('usr/include/stdio.h', b'changed target header')
            self.assertNotEqual(key, cb.incremental_build_dir(
                args, self.meta, source, self.root, 'sha256:one', [],
            ))

    def test_sdk_paths_reject_remote_shell_and_traversal(self):
        for value in ['/', '/etc', '/usr/../etc', '/usr/a;touch x', '/opt/a b']:
            with self.assertRaises(ValueError):
                cb.validate_extra_dir(value)
        self.assertEqual(cb.validate_extra_dir('/usr/local/cuda-12.6'), '/usr/local/cuda-12.6')


if __name__ == '__main__':
    unittest.main()
