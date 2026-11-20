#!/usr/bin/env python3
"""Profile-driven cross compilation. Requires Python 3.9+, no third-party modules."""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import struct
import subprocess
import sys
import tempfile
import uuid

ROOT = Path(__file__).resolve().parents[1]
WORK = ROOT / 'work'
DEFAULT_IMAGE = 'autonomy.platform.x86_64'
ABIS = {
    'aarch64': ('aarch64-linux-gnu', 2, 183, '/lib/ld-linux-aarch64.so.1'),
    'armhf': ('arm-linux-gnueabihf', 1, 40, '/lib/ld-linux-armhf.so.3'),
}
TOKEN = re.compile(r'[a-z0-9]+(?:-[a-z0-9]+)*\Z')


def profile_dirs():
    return sorted(p for p in (ROOT / 'profiles').iterdir() if p.is_dir())


def validate_profile(meta, name):
    if not isinstance(meta, dict):
        raise ValueError(f'{name}: profile must be a JSON object')
    for key in ('platform', 'vendor', 'model', 'stack', 'version', 'arch', 'family',
                'triple', 'distro', 'distro_version', 'gcc_major'):
        if not isinstance(meta.get(key), str) or not meta[key]:
            raise ValueError(f'{name}: missing/string field required: {key}')
    for key in ('vendor', 'model', 'stack', 'family'):
        if not TOKEN.fullmatch(meta[key]):
            raise ValueError(f'{name}: invalid {key}')
    if meta['vendor'] not in ('rockchip', 'nvidia') or meta['stack'] not in ('ubuntu', 'debian', 'jetpack'):
        raise ValueError(f'{name}: unsupported vendor/stack; extend validator when adding a new one')
    if not re.fullmatch(r'[0-9]+', meta['version']):
        raise ValueError(f'{name}: version must be an ID digit token')
    expected = f"{meta['vendor']}-{meta['model']}-{meta['stack']}{meta['version']}"
    if expected != name or meta['platform'] != name:
        raise ValueError(f'{name}: directory/platform must equal {expected}')
    if meta['arch'] not in ABIS or meta['triple'] != ABIS[meta['arch']][0]:
        raise ValueError(f'{name}: unsupported architecture/triple')
    if meta['gcc_major'] != {'aarch64': '11', 'armhf': '12'}[meta['arch']]:
        raise ValueError(f'{name}: unsupported GCC baseline for {meta["arch"]}')
    expected_stack = {'ubuntu': ('ubuntu', '22.04', '2204'),
                      'debian': ('debian', '12', '12'),
                      'jetpack': ('ubuntu', '22.04', '6')}[meta['stack']]
    if (meta['distro'], meta['distro_version'], meta['version']) != expected_stack:
        raise ValueError(f'{name}: stack/version/OS mismatch')
    family_arch = {('rockchip', 'rk3588'): 'aarch64',
                   ('rockchip', 'rk3506'): 'armhf', ('nvidia', 'orin'): 'aarch64'}
    if family_arch.get((meta['vendor'], meta['family'])) != meta['arch']:
        raise ValueError(f'{name}: unsupported chip family/ABI')
    if not (ROOT / 'cmake/linux.cmake').is_file():
        raise ValueError('missing common Linux toolchain')
    platform_file = ROOT / 'cmake/platform' / f"{meta['vendor']}-{meta['family']}.cmake"
    if not platform_file.is_file():
        raise ValueError(f'{name}: missing {platform_file}')
    return meta


def load_profile(name):
    if not name or not TOKEN.fullmatch(name):
        raise ValueError('select a valid Profile with --profile or PROFILE')
    directory = ROOT / 'profiles' / name
    with (directory / 'profile.yaml').open() as stream:
        meta = validate_profile(json.load(stream), name)
    toolchain = directory / 'toolchain.cmake'
    if not toolchain.is_file():
        raise ValueError(f'missing {toolchain}')
    # These small generated entry points have a fixed contract; reject stale copies.
    if toolchain.read_text() != toolchain_text(meta):
        raise ValueError(f'{toolchain}: does not match profile metadata (see docs/profiles.md)')
    return meta


def toolchain_text(meta):
    return (f'set(CROSS_TRIPLE "{meta["triple"]}")\n'
            f'set(CMAKE_SYSTEM_PROCESSOR "{"armv7l" if meta["arch"] == "armhf" else "aarch64"}")\n'
            f'include("${{CMAKE_CURRENT_LIST_DIR}}/../../cmake/platform/{meta["vendor"]}-{meta["family"]}.cmake")\n'
            'include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/linux.cmake")\n')


def run(command, **kwargs):
    print('+ ' + shlex.join(str(x) for x in command), flush=True)
    return subprocess.run([str(x) for x in command], check=True, **kwargs)


@contextmanager
def profile_lock(work, name):
    work.mkdir(parents=True, exist_ok=True)
    lock = work / f'.{name}.lock'
    try:
        lock.mkdir()
    except FileExistsError:
        raise ValueError(f'{name} is locked: {lock}; remove only after confirming no active operation') from None
    try:
        (lock / 'owner').write_text(f'pid={os.getpid()}\n')
        yield
    finally:
        (lock / 'owner').unlink(missing_ok=True)
        lock.rmdir()


def normalize_links(root):
    root = root.resolve()
    links = []
    for directory, dirs, files in os.walk(root, followlinks=False):
        for name in dirs + files:
            path = Path(directory) / name
            if path.is_symlink():
                links.append(path)
    # Normalize every absolute link before resolving any link chains.
    for path in links:
        target = os.readlink(path)
        if target.startswith('/'):
            relative = os.path.relpath(root / target.lstrip('/'), path.parent)
            path.unlink()
            path.symlink_to(relative)
    for path in links:
        try:
            resolved = path.resolve()
        except (RuntimeError, OSError) as error:
            raise ValueError(f'cyclic/invalid symlink: {path}: {error}') from error
        if not resolved.is_relative_to(root):
            raise ValueError(f'symlink escapes sysroot: {path}')


def safe_file(root, relative):
    path = root / relative.lstrip('/')
    try:
        resolved = path.resolve()
    except (RuntimeError, OSError) as error:
        raise ValueError(f'invalid sysroot path: {relative}') from error
    if not resolved.is_relative_to(root.resolve()) or not resolved.is_file():
        raise ValueError(f'missing/escaping sysroot file: {relative}')
    return resolved


def check_elf(path, meta):
    with path.open('rb') as stream:
        header = stream.read(64)
    _, elf_class, machine, _ = ABIS[meta['arch']]
    if len(header) < 52 or header[:4] != b'\x7fELF' or header[4:6] != bytes([elf_class, 1]):
        raise ValueError(f'wrong ELF class/endianness: {path}')
    if struct.unpack_from('<H', header, 18)[0] != machine:
        raise ValueError(f'wrong ELF machine: {path}')
    if meta['arch'] == 'armhf' and not struct.unpack_from('<I', header, 36)[0] & 0x400:
        raise ValueError(f'ARM hard-float ABI required: {path}')


def os_release(path):
    values = {}
    for line in path.read_text().splitlines():
        if '=' in line and not line.startswith('#'):
            key, value = line.split('=', 1)
            parsed = shlex.split(value)
            if len(parsed) == 1:
                values[key] = parsed[0]
    return values


def verify_tree(root, meta):
    root = root.resolve()
    # Verification never rewrites an existing tree.
    for directory, dirs, files in os.walk(root, followlinks=False):
        for name in dirs + files:
            path = Path(directory) / name
            if path.is_symlink():
                if os.path.isabs(os.readlink(path)):
                    raise ValueError(f'absolute link requires normalization during import: {path}')
                try:
                    inside = path.resolve().is_relative_to(root)
                except (RuntimeError, OSError) as error:
                    raise ValueError(f'invalid link: {path}') from error
                if not inside:
                    raise ValueError(f'symlink escapes sysroot: {path}')
    release = os_release(safe_file(root, 'etc/os-release'))
    if (release.get('ID'), release.get('VERSION_ID')) != (meta['distro'], meta['distro_version']):
        raise ValueError(f'OS mismatch: expected {meta["distro"]} {meta["distro_version"]}, got {release}')
    safe_file(root, 'usr/include/stdio.h')
    triple = meta['triple']
    safe_file(root, f'usr/lib/{triple}/crt1.o')
    for relative in (f'usr/lib/{triple}/libc.so.6', f'usr/lib/{triple}/libstdc++.so.6', ABIS[meta['arch']][3]):
        check_elf(safe_file(root, relative), meta)
    if meta['stack'] == 'jetpack':
        release_text = safe_file(root, 'etc/nv_tegra_release').read_text()
        if not re.search(r'^# R36\b', release_text, re.MULTILINE):
            raise ValueError('JetPack 6 profile requires a Jetson Linux R36 sysroot')
    manifest = root / '.cross-build.json'
    if manifest.exists():
        if json.loads(safe_file(root, '.cross-build.json').read_text()).get('platform') != meta['platform']:
            raise ValueError('sysroot manifest Profile mismatch')
    return release


def publish_tree(staging, destination):
    backup = destination.with_name(destination.name + '.previous-' + uuid.uuid4().hex)
    had_old = destination.exists()
    if had_old:
        os.replace(destination, backup)
    try:
        os.replace(staging, destination)
    except OSError:
        if had_old:
            os.replace(backup, destination)
        raise
    if had_old:
        print(f'Previous sysroot retained: {backup}')


def validate_extra_dir(value):
    if not re.fullmatch(r'/(?:usr|opt)/[a-zA-Z0-9_./+-]+', value) or '..' in Path(value).parts:
        raise ValueError(f'extra directory must be a simple absolute /usr/... or /opt/... path: {value}')
    return value.rstrip('/')


def collect_sysroot(args, meta):
    if not args.host or not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9.-]*', args.host):
        raise ValueError('HOST/--host must be a hostname or IPv4 address')
    if not args.user or not re.fullmatch(r'[a-zA-Z_][a-zA-Z0-9_-]*', args.user):
        raise ValueError('USER/--user must be a simple SSH user name')
    remote = f'{args.user}@{args.host}'
    paths = ['/lib', '/usr/lib', '/usr/include', '/etc/os-release']
    optional = ['/lib64', '/usr/share/pkgconfig', '/usr/lib64']
    if meta['stack'] == 'jetpack':
        paths.append('/etc/nv_tegra_release')
    for value in args.extra_dir:
        paths.append(validate_extra_dir(value))
    # Fixed shell syntax; each remote pathname is quoted independently.
    for path in optional:
        result = subprocess.run(['ssh', remote, 'test -e ' + shlex.quote(path)], check=False)
        if result.returncode == 0:
            paths.append(path)
        elif result.returncode != 1:
            raise ValueError(f'SSH failed while checking {path}, exit={result.returncode}')
    parent = WORK / 'sysroots'
    parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f'.{meta["platform"]}-', dir=parent))
    try:
        run(['rsync', '-rlpt', '--relative', '--protect-args', '-e', 'ssh',
             *[f'{remote}:/./{path.lstrip("/")}' for path in paths], str(staging) + '/'])
        normalize_links(staging)
        release = verify_tree(staging, meta)
        manifest = {'platform': meta['platform'], 'source': remote, 'paths': paths,
                    'collected_at': datetime.now(timezone.utc).isoformat(), 'os_release': release}
        if meta['stack'] == 'jetpack':
            manifest['nv_tegra_release'] = (staging / 'etc/nv_tegra_release').read_text()
        (staging / '.cross-build.json').write_text(json.dumps(manifest, indent=2) + '\n')
        publish_tree(staging, parent / meta['platform'])
    finally:
        if staging.exists():
            shutil.rmtree(staging)


def sysroot_fingerprint(sysroot):
    digest = hashlib.sha256()
    for path in sorted(sysroot.rglob('*')):
        relative = path.relative_to(sysroot)
        if relative.parts == ('.cross-build.json',):
            continue
        digest.update(os.fsencode(relative) + b'\0')
        if path.is_symlink():
            digest.update(b'link:' + os.fsencode(os.readlink(path)) + b'\0')
        elif path.is_file():
            content = hashlib.sha256()
            with path.open('rb') as stream:
                mode = os.fstat(stream.fileno()).st_mode & 0o7777
                for block in iter(lambda: stream.read(1024 * 1024), b''):
                    content.update(block)
            digest.update(b'file:' + mode.to_bytes(2, 'big') + content.digest())
        else:
            digest.update(b'dir:')
    return digest.hexdigest()


def incremental_build_dir(args, meta, source, sysroot, image_id, cmake_args):
    profile = meta['platform']
    family = f"{meta['vendor']}-{meta['family']}.cmake"
    toolchain_files = [Path(__file__), ROOT / 'profiles' / profile / 'toolchain.cmake',
                       ROOT / 'cmake/linux.cmake', ROOT / 'cmake/platform' / family]
    inputs = {
        'source': str(source), 'profile': meta, 'sysroot': sysroot_fingerprint(sysroot),
        'image_id': image_id, 'docker_platform': args.docker_platform,
        'prefix': args.install_prefix or '/install', 'cmake_args': cmake_args,
        'toolchain': [hashlib.sha256(path.read_bytes()).hexdigest() for path in toolchain_files],
    }
    fingerprint = hashlib.sha256(json.dumps(inputs, sort_keys=True).encode()).hexdigest()
    return WORK / 'incremental' / profile / fingerprint / 'build'


def local_image_id(image, platform):
    result = subprocess.run(
        ['docker', 'image', 'inspect', '--format', '{{.Id}} {{.Os}}/{{.Architecture}}', image],
        capture_output=True, text=True, check=False,
    )
    fields = result.stdout.split()
    if (result.returncode == 0 and len(fields) == 2 and fields[1] == platform
            and re.fullmatch(r'sha256:[0-9a-f]{64}', fields[0])):
        return fields[0]
    return None


def source_date_epoch(source):
    if shutil.which('git'):
        result = subprocess.run(['git', '-C', str(source), 'log', '-1', '--format=%ct'],
                                capture_output=True, text=True, check=False)
        if result.returncode == 0 and result.stdout.strip().isdigit():
            return result.stdout.strip()
    return '0'


def build_project(args, meta):
    source = Path(args.source).expanduser().resolve()
    if not (source / 'CMakeLists.txt').is_file():
        raise ValueError(f'SOURCE must contain CMakeLists.txt: {source}')
    sysroot = WORK / 'sysroots' / meta['platform']
    verify_tree(sysroot, meta)
    if args.output_dir:
        workspace = Path(args.output_dir).expanduser().resolve()
        output, install = workspace / 'build', workspace / 'install'
    else:
        output = WORK / 'build' / meta['platform']
        install = WORK / 'install' / meta['platform']
    extra = args.cmake_args
    if extra and extra[0] == '--':
        extra = extra[1:]
    image = args.image or DEFAULT_IMAGE
    epoch = None
    if args.incremental:
        epoch = source_date_epoch(source)
        image_id = local_image_id(image, args.docker_platform)
        if image_id:
            output = incremental_build_dir(args, meta, source, sysroot, image_id, extra)
            image = image_id
            action = 'Reusing' if (output / 'CMakeCache.txt').exists() else 'Creating'
            print(f'{action} incremental build cache: {output}', flush=True)
        else:
            print('No matching local image ID; using an isolated build.', flush=True)
    for path in (output, install):
        path.mkdir(parents=True, exist_ok=True)
    # Docker --mount uses comma-delimited fields and cannot represent these paths.
    for path in (source, sysroot, output, install, ROOT):
        if ',' in str(path):
            raise ValueError('Docker mount paths cannot contain commas')
    triple = meta['triple']
    pkg_paths = f'/sysroot/usr/lib/{triple}/pkgconfig:/sysroot/usr/lib/pkgconfig:/sysroot/usr/share/pkgconfig'
    command = ['docker', 'run', '--rm', '--platform', args.docker_platform,
               '--user', f'{os.getuid()}:{os.getgid()}', '--env', 'HOME=/tmp',
               '--env', 'DESTDIR=',
               '--entrypoint', '/bin/sh', '--workdir', '/build',
               '--env', 'PKG_CONFIG_PATH=', '--env', 'PKG_CONFIG_SYSROOT_DIR=/sysroot',
               '--env', f'PKG_CONFIG_LIBDIR={pkg_paths}']
    if epoch is not None:
        command += ['--env', f'SOURCE_DATE_EPOCH={epoch}']
    for host, target, readonly in ((source, '/src', True), (sysroot, '/sysroot', True),
                                    (ROOT, '/cross', True), (output, '/build', False),
                                    (install, '/install', False)):
        command += ['--mount', f'type=bind,src={host},dst={target}' + (',readonly' if readonly else '')]
    # Exact argument boundaries survive spaces and shell metacharacters.
    script = ('set -eu\n'
              'prefix=$1\nshift\n'
              f'for tool in {triple}-gcc {triple}-g++; do\n'
              '  command -v "$tool" >/dev/null || {\n'
              '    echo "Missing $tool. Prepare the shared image in docker/ or set IMAGE to a compatible image." >&2\n'
              '    exit 1\n'
              '  }\n'
              'done\n'
              f'{triple}-gcc --version\n'
              'cmake -S /src -B /build -G Ninja -DCMAKE_BUILD_TYPE=Release '
              f'-DCMAKE_TOOLCHAIN_FILE=/cross/profiles/{meta["platform"]}/toolchain.cmake '
              '"-DCMAKE_INSTALL_PREFIX=$prefix" "$@"\n'
              'cmake --build /build --parallel\n')
    if args.command == 'install':
        if args.install_prefix:
            script += 'DESTDIR=/install cmake --install /build --prefix "$prefix"\n'
        else:
            script += 'cmake --install /build\n'
    command += [image, '-c', script, 'cross-build',
                args.install_prefix or '/install', *extra]
    run(command)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['list', 'check', 'sysroot', 'verify', 'build', 'install'])
    parser.add_argument('--profile', default=os.environ.get('PROFILE'))
    parser.add_argument('--host', default=os.environ.get('HOST'))
    parser.add_argument('--user', default=os.environ.get('USER'))
    parser.add_argument('--source', default=os.environ.get('SOURCE') or str(ROOT.parents[1]))
    parser.add_argument('--output-dir', help='Use build/ and install/ under this workspace')
    parser.add_argument('--install-prefix', help='Stage an absolute target install prefix below install/')
    parser.add_argument('--incremental', action='store_true', help='Reuse a compatible CMake build directory')
    parser.add_argument('--image', default=os.environ.get('IMAGE') or None)
    parser.add_argument('--docker-platform', default=os.environ.get('DOCKER_PLATFORM') or 'linux/amd64',
                        choices=['linux/amd64', 'linux/arm64'])
    parser.add_argument('--extra-dir', action='append', default=[])
    parser.add_argument('--refresh-sysroot', action='store_true', help='Collect the sysroot before install')
    # parse_known_args keeps arbitrary CMake -D arguments without shell re-parsing.
    args, extra = parser.parse_known_args(argv)
    args.cmake_args = extra
    if extra and (args.command not in ('build', 'install') or extra[0] != '--'):
        parser.error('CMake arguments are accepted only after -- for build/install')
    if args.install_prefix and (
            args.command != 'install' or not args.output_dir or args.install_prefix == '/'
            or not args.install_prefix.startswith('/')
            or os.path.normpath(args.install_prefix) != args.install_prefix
            or args.install_prefix.startswith('//')):
        parser.error('--install-prefix requires install, --output-dir and an absolute non-root prefix')
    if args.refresh_sysroot and args.command != 'install':
        parser.error('--refresh-sysroot requires install')
    if args.incremental and (args.command not in ('build', 'install') or not args.output_dir):
        parser.error('--incremental requires build or install and a separate --output-dir')
    try:
        if args.command in ('list', 'check'):
            for directory in profile_dirs():
                meta = load_profile(directory.name)
                print(f'{meta["platform"]:42} {meta["triple"]:24} {meta["distro"]} {meta["distro_version"]} gcc-{meta["gcc_major"]}')
            return 0
        meta = load_profile(args.profile)
        with profile_lock(WORK, meta['platform']):
            if args.command == 'sysroot':
                collect_sysroot(args, meta)
            elif args.command == 'verify':
                verify_tree(WORK / 'sysroots' / meta['platform'], meta)
                print('Sysroot structural checks passed; runtime/SDK compatibility is not tested.')
            else:
                if args.refresh_sysroot:
                    collect_sysroot(args, meta)
                build_project(args, meta)
        return 0
    except (ValueError, OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f'cross-build: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
