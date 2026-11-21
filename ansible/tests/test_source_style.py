"""Exercise the build style gate against a miniature first-party checkout."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[2]
FORMATTER = shutil.which('clang-format')


@unittest.skipUnless(FORMATTER, 'requires clang-format')
class SourceStyleTest(unittest.TestCase):
    def test_all_diagnostics_block_build_and_check_does_not_mutate_source(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            checker = root / 'tools/python/clang_format_sources.py'
            checker.parent.mkdir(parents=True)
            shutil.copy2(REPO / 'tools/python/clang_format_sources.py', checker)
            (root / '.clang-format').write_text('BasedOnStyle: Google\n')
            subprocess.run(['git', 'init', '-q', str(root)], check=True)
            (root / 'first.cpp').write_text('int first(){return 0;}\n')
            (root / 'new.cpp').write_text('int second(){return 0;}\n')
            (root / 'vendor').mkdir()
            (root / 'vendor/ignored.cpp').write_text('int ignored(){return 0;}\n')
            subprocess.run(['git', '-C', str(root), 'add', 'first.cpp', 'vendor'], check=True)
            report = root / 'style.log'
            command = [sys.executable, str(checker), '--check', '--report', str(report)]
            env = {**os.environ, 'XDG_CACHE_HOME': str(root / 'cache'), 'AUTONOMY_STYLE_CACHE': '1'}
            failed = subprocess.run(command, capture_output=True, text=True,
                                    env={**env, 'AUTONOMY_LOG_COLOR': 'always'})
            self.assertNotEqual(failed.returncode, 0, failed.stderr)
            self.assertIn('\x1b[31m', failed.stderr)
            self.assertNotIn('\x1b[', report.read_text())
            self.assertIn('first.cpp:', report.read_text())
            self.assertIn('new.cpp:', report.read_text())
            self.assertNotIn('ignored.cpp:', report.read_text())
            self.assertEqual((root / 'first.cpp').read_text(), 'int first(){return 0;}\n')
            subprocess.run([sys.executable, str(checker), 'first.cpp', 'new.cpp'],
                           check=True, stdout=subprocess.DEVNULL, env=os.environ.copy())
            repaired = subprocess.run(command, capture_output=True, text=True, env=env)
            self.assertEqual(repaired.returncode, 0, repaired.stderr)
            warm = subprocess.run([*command[:3], '-o', str(report)], capture_output=True, text=True, env=env)
            self.assertEqual(warm.returncode, 0, warm.stderr)
            self.assertIn('0 checked, 2 cached', warm.stderr)
            (root / 'new.cpp').write_text('int second(){return 0;}\n')
            edited = subprocess.run(command, capture_output=True, text=True, env=env)
            self.assertNotEqual(edited.returncode, 0)
            self.assertIn('1 checked, 1 cached', edited.stderr)
            (root / '.clang-format').write_text('BasedOnStyle: LLVM\n')
            rules_changed = subprocess.run(command, capture_output=True, text=True, env=env)
            self.assertIn('2 checked, 0 cached', rules_changed.stderr)
