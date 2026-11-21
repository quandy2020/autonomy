#!/usr/bin/env python3
"""Check or format first-party C/C++ files using the repository's Google-based rules."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time


ROOT = Path(__file__).resolve().parents[2]
EXTENSIONS = {'.c', '.cc', '.cpp', '.cxx', '.h', '.hh', '.hpp', '.hxx', '.h++',
              '.inl', '.ipp', '.tpp', '.cu', '.cuh'}
EXCLUDED_DIRS = {'thirdparty', 'third_party', 'vendor', 'external', 'node_modules', '_deps', 'build', 'install'}


def git_paths(root):
    result = subprocess.run(
        ['git', '-C', str(root), 'ls-files', '-z', '--cached', '--others', '--exclude-standard'],
        check=True, stdout=subprocess.PIPE,
    )
    return {Path(os.fsdecode(name)) for name in result.stdout.split(b'\0') if name}


def source_files(root, scopes):
    files = git_paths(root)
    # Autolink is first-party. Vendored gitlinks must not be traversed.
    if Path('autolink') in files and not any(
            path.parts[0] == 'autolink' and len(path.parts) > 1 for path in files):
        autolink = root / 'autolink'
        if not (autolink / '.git').exists():
            raise ValueError('Autolink is not checked out. Run git submodule update --init -- autolink.')
        files.update(Path('autolink') / path for path in git_paths(autolink))
    return sorted(
        path for path in files
        if path.suffix.lower() in EXTENSIONS
        and not any(part in EXCLUDED_DIRS for part in path.parts[:-1])
        and not path.name.endswith(('.pb.cc', '.pb.h'))
        and not (root / path).is_symlink()
        and (root / path).is_file()
        and (not scopes or any(path == scope or scope in path.parents for scope in scopes))
    )


def find_clang_format():
    configured = os.environ.get('AUTONOMY_CLANG_FORMAT')
    candidates = (configured,) if configured else ('clang-format', *(f'clang-format-{n}' for n in range(24, 12, -1)))
    for name in candidates:
        path = shutil.which(name)
        if path:
            return path
    raise ValueError('clang-format is required on the build host. Install it and rerun the style check.')


def style_keys(root, files, tool_key):
    configurations = {}
    keys = {}
    for path in files:
        directory = (root / path).parent
        if directory not in configurations:
            digest = hashlib.sha256(tool_key)
            for parent in (directory, *directory.parents):
                for name in ('.clang-format', '_clang-format', '.clang-format-ignore'):
                    config = parent / name
                    if config.is_file():
                        digest.update(os.fsencode(config) + b'\0' + hashlib.sha256(config.read_bytes()).digest())
            configurations[directory] = digest.digest()
        keys[str(path)] = hashlib.sha256(
            configurations[directory] + os.fsencode(path) + b'\0' + (root / path).read_bytes(),
        ).hexdigest()
    return keys


def formatter_identity():
    tool = find_clang_format()
    version = subprocess.check_output([tool, '--version'], text=True).strip()
    expected = os.environ.get('AUTONOMY_CLANG_FORMAT_VERSION')
    if expected and not re.search(r'\bversion\s+' + re.escape(expected) + r'(?:\s|$)', version):
        raise ValueError(f'expected clang-format {expected}; found {version}')
    return tool, version


def show_diagnostic(message, color, stream=sys.stderr):
    """Color console output only. The diagnostic report keeps the original bytes."""
    mode = os.environ.get('AUTONOMY_LOG_COLOR', 'auto')
    if mode == 'always' or (mode == 'auto' and stream.isatty() and os.environ.get('TERM') != 'dumb'
                            and not os.environ.get('NO_COLOR')):
        message = f'\033[{color}m{message}\033[0m'
    print(message, file=stream)


def format_sources(root, files, check, report_path=None):
    start = time.monotonic()
    tool, version = formatter_identity()
    flags = ['--dry-run', '--Werror', '--ferror-limit=0'] if check else ['-i']
    cache = None
    cached = {}
    if check and os.environ.get('AUTONOMY_STYLE_CACHE') != '0':
        identity = hashlib.sha256(os.fsencode(root)).hexdigest()
        cache_root = Path(os.environ.get('XDG_CACHE_HOME') or Path.home() / '.cache')
        cache = cache_root / 'autonomy/style' / (identity + '.json')
        try:
            previous = json.loads(cache.read_text())
            if isinstance(previous, dict):
                cached = previous
        except (OSError, ValueError):
            pass
    resolved_tool = Path(tool).resolve()
    info = resolved_tool.stat()
    tool_key = repr((version, tuple(flags), str(resolved_tool), info.st_dev, info.st_ino,
                     info.st_size, info.st_mtime_ns, info.st_ctime_ns)).encode()
    keys = style_keys(root, files, tool_key) if cache else {}
    pending = [path for path in files if not cache or cached.get(str(path)) != keys[str(path)]]
    passed = {str(path): keys[str(path)] for path in files if cache and cached.get(str(path)) == keys[str(path)]}
    failed = False
    diagnostics = 0
    bad_files = set()
    if check and report_path is None:
        fd, name = tempfile.mkstemp(prefix='autonomy-style-', suffix='.log')
        os.close(fd)
        report_path = Path(name)
        temporary = True
    else:
        temporary = False
    report = report_path.open('wb') if check else None
    try:
        # Bound argument size while continuing through every batch after errors.
        for offset in range(0, len(pending), 128):
            batch = pending[offset:offset + 128]
            result = subprocess.run(
                [tool, '--style=file', *flags, '--', *map(str, batch)],
                cwd=root, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            )
            failed = failed or result.returncode != 0 or bool(result.stderr)
            if cache and not result.returncode and not result.stderr:
                for name, current in style_keys(root, batch, tool_key).items():
                    if current == keys[name]:
                        passed[name] = current
            if report is None:
                sys.stdout.buffer.write(result.stdout)
                sys.stderr.buffer.write(result.stderr)
                continue
            report.write(result.stdout)
            report.write(result.stderr)
            for line in result.stderr.splitlines():
                location = re.match(rb'^(.+?):\d+:\d+: (error|warning):', line)
                if location:
                    bad_files.add(location[1])
                if location or b'error:' in line.lower() or b'warning:' in line.lower():
                    diagnostics += 1
                    if diagnostics <= 20:
                        warning = location[2] == b'warning' if location else b'warning:' in line.lower()
                        show_diagnostic(os.fsdecode(line), 33 if warning else 31)
    finally:
        if report is not None:
            report.close()
    if cache and pending:
        try:
            cached.update(passed)
            cache.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryDirectory(dir=cache.parent) as temporary_cache:
                staged = Path(temporary_cache) / 'cache.json'
                staged.write_text(json.dumps(cached))
                os.replace(staged, cache)
        except OSError as error:
            show_diagnostic(f'C/C++ style cache unavailable: {error}', 33)
    if check:
        show_diagnostic(f'Style check: {len(pending)} checked, {len(files) - len(pending)} cached, '
                        f'{time.monotonic() - start:.2f}s.', 36)
    if failed:
        show_diagnostic(f'C/C++ style check failed: {diagnostics} diagnostics in {len(bad_files)} files.', 31)
        if report_path is not None:
            print(f'Complete diagnostics: {report_path}', file=sys.stderr)
        print('Fix every diagnostic and rerun before building.', file=sys.stderr)
        print('To format a specific file: python3 tools/python/clang_format_sources.py path/to/file.cpp', file=sys.stderr)
        return 1
    if temporary:
        report_path.unlink()
    show_diagnostic(f'{"Checked" if check else "Formatted"} {len(files)} first-party C/C++ files.',
                    32, stream=sys.stdout)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='Report every violation and exit nonzero')
    mode.add_argument('--dry-run', action='store_true', help='List the selected files')
    mode.add_argument('--version', action='store_true', help='Print the resolved formatter version')
    parser.add_argument('-o', '--report', type=Path, help='Write the complete check report to this path')
    parser.add_argument('paths', nargs='*', help='Optional files or directories relative to the repository root')
    args = parser.parse_args(argv)
    if args.report is not None and not args.check:
        parser.error('--report requires --check')
    try:
        if args.version:
            print(formatter_identity()[1])
            return 0
        scopes = [Path(name) for name in args.paths]
        if any(path.is_absolute() or '..' in path.parts for path in scopes):
            raise ValueError('paths must be relative to the repository root')
        if not (ROOT / '.clang-format').is_file():
            raise ValueError(f'missing repository style configuration: {ROOT / ".clang-format"}')
        files = source_files(ROOT, scopes)
        if not files:
            raise ValueError('no first-party C/C++ sources selected')
        if args.dry_run:
            print(*files, sep='\n')
            return 0
        return format_sources(ROOT, files, check=args.check, report_path=args.report)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        show_diagnostic(f'C/C++ style check: {error}', 31)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
