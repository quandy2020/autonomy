#!/usr/bin/env python3

###############################################################################
# Copyright 2024 The OpenRobotic Beginner Authors (duyongquan). All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

"""Install or uninstall Docker on Ubuntu system.

Examples:
    # Install Docker (default)
    python3 install_docker.py

    # Install Docker explicitly
    python3 install_docker.py install

    # Uninstall Docker
    python3 install_docker.py uninstall
"""

import argparse
import os
import platform
import re
import subprocess
import sys
from pathlib import Path

try:
    from print_color import print_error, print_info, print_warning
except ImportError:
    def print_error(msg): print(f"ERROR: {msg}", file=sys.stderr)
    def print_warning(msg): print(f"WARNING: {msg}", file=sys.stderr)
    def print_info(msg): print(f"INFO: {msg}")


def run_command(cmd, check=True, shell=False, capture=False):
    """Run a shell command. Stream output by default; print it on failure."""
    if isinstance(cmd, str) and not shell:
        cmd = cmd.split()
    result = subprocess.run(
        cmd,
        shell=shell,
        check=False,
        capture_output=capture,
        text=True,
    )
    if result.returncode != 0:
        if capture:
            if result.stdout:
                print(result.stdout, end='')
            if result.stderr:
                print(result.stderr, end='', file=sys.stderr)
        if check:
            raise subprocess.CalledProcessError(
                result.returncode, result.args, result.stdout, result.stderr
            )
    return result


def run_sudo_command(cmd, check=True, shell=False, capture=False):
    """Run a command with sudo."""
    if isinstance(cmd, str) and not shell:
        cmd = cmd.split()
    sudo_cmd = ['sudo', 'env', 'DEBIAN_FRONTEND=noninteractive'] + (
        cmd if isinstance(cmd, list) else [cmd]
    )
    return run_command(sudo_cmd, check=check, shell=shell, capture=capture)


def _raise_apt_error(action, exc):
    """Print apt/dpkg hints and re-raise."""
    print_error(f"apt {action} failed (exit {exc.returncode}).")
    combined = f"{exc.stdout or ''}{exc.stderr or ''}"
    if 'lock' in combined.lower():
        print_error(
            "Another package manager may be running. "
            "Close other apt/synaptic windows and retry."
        )
    elif 'NO_PUBKEY' in combined or 'GPG error' in combined:
        print_error(
            "A third-party apt repository has an invalid or missing GPG key. "
            "Fix or disable that source under /etc/apt/sources.list.d/ and retry."
        )
    raise exc


def apt_update():
    """Refresh apt package lists."""
    try:
        run_sudo_command(['apt-get', 'update'])
    except subprocess.CalledProcessError as exc:
        _raise_apt_error('update', exc)


def apt_install(packages):
    """Install packages with apt-get."""
    if not packages:
        return
    try:
        run_sudo_command(['apt-get', 'install', '-y', '--no-install-recommends', *packages])
    except subprocess.CalledProcessError as exc:
        _raise_apt_error('install', exc)


def _parse_kernel_major_version(kernel_version: str) -> int:
    """Parse the major kernel version from uname release (e.g. 6.17.0-1025-oem -> 6)."""
    match = re.match(r'^(\d+)', kernel_version)
    return int(match.group(1)) if match else 0


def _overlay_supported(kernel_version: str) -> bool:
    """Return True if the overlayfs module is available for this kernel."""
    overlay_dir = Path(f'/lib/modules/{kernel_version}/kernel/fs/overlayfs')
    if overlay_dir.is_dir() and any(overlay_dir.glob('overlay.ko*')):
        return True
    result = run_command(['modinfo', 'overlay'], check=False, capture=True)
    return result.returncode == 0


def ensure_overlay_module():
    """Load overlay module when available (required for Docker overlay2)."""
    kernel_version = platform.release()
    main_kernel_version = _parse_kernel_major_version(kernel_version)

    if main_kernel_version >= 4:
        if _overlay_supported(kernel_version):
            print_info(f"Kernel {kernel_version} supports overlay2.")
            run_sudo_command(['modprobe', 'overlay'], check=False)
        else:
            print_warning(
                f"Kernel {kernel_version} is 4+; overlay2 is expected. "
                "Skipping legacy AUFS packages."
            )
    else:
        print_info("Kernel version is lower than 4; trying to install aufs.")
        apt_update()
        apt_install([
            f'linux-image-extra-{kernel_version}',
            'linux-image-extra-virtual',
        ])


def get_ubuntu_codename():
    """Return Ubuntu release codename (e.g. noble)."""
    try:
        result = run_command(['lsb_release', '-cs'], check=True, capture=True)
        codename = result.stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        print_error("lsb_release not found. Cannot determine Ubuntu codename.")
        sys.exit(1)
    if not codename:
        print_error("Could not determine Ubuntu codename.")
        sys.exit(1)
    return codename


def _docker_deb_arch(machine_arch: str) -> str:
    """Map platform.machine() to Docker apt repository arch."""
    mapping = {
        'x86_64': 'amd64',
        'aarch64': 'arm64',
        'arm64': 'arm64',
    }
    deb_arch = mapping.get(machine_arch)
    if deb_arch is None:
        print_error(f"Unsupported machine architecture: {machine_arch}")
        sys.exit(1)
    return deb_arch


def setup_docker_apt_repo(codename: str, deb_arch: str):
    """Configure Docker's official apt repository (keyring + sources list)."""
    keyring_path = '/etc/apt/keyrings/docker.asc'
    sources_path = '/etc/apt/sources.list.d/docker.list'
    repo_line = (
        f"deb [arch={deb_arch} signed-by={keyring_path}] "
        f"https://download.docker.com/linux/ubuntu {codename} stable"
    )

    run_sudo_command(['install', '-m', '0755', '-d', '/etc/apt/keyrings'])
    run_sudo_command([
        'curl', '-fsSL', 'https://download.docker.com/linux/ubuntu/gpg',
        '-o', keyring_path,
    ])
    run_sudo_command(['chmod', 'a+r', keyring_path])

    write_repo = subprocess.run(
        ['sudo', 'tee', sources_path],
        input=repo_line + '\n',
        text=True,
        capture_output=True,
        check=False,
    )
    if write_repo.returncode != 0:
        if write_repo.stdout:
            print(write_repo.stdout, end='')
        if write_repo.stderr:
            print(write_repo.stderr, end='', file=sys.stderr)
        raise subprocess.CalledProcessError(
            write_repo.returncode, write_repo.args, write_repo.stdout, write_repo.stderr
        )


def install_docker_packages():
    """Install Docker Engine packages from Docker's apt repository."""
    apt_update()
    apt_install([
        'docker-ce',
        'docker-ce-cli',
        'containerd.io',
        'docker-buildx-plugin',
        'docker-compose-plugin',
    ])


def configure_docker_group():
    """Create docker group and add the current user."""
    run_sudo_command(['groupadd', 'docker'], check=False)

    current_user = os.getenv('USER') or os.getenv('USERNAME')
    if not current_user:
        print_warning("Could not detect current user; skip adding to docker group.")
        return

    run_sudo_command(['gpasswd', '-a', current_user, 'docker'])
    print_info(f"User {current_user} has been added to the docker group.")
    print_info("Log out and back in, or run: newgrp docker")


def install():
    """Main installation function."""
    machine_arch = platform.machine()
    deb_arch = _docker_deb_arch(machine_arch)

    ensure_overlay_module()

    # ca-certificates + curl only; HTTPS apt transport is built into modern Ubuntu apt.
    apt_update()
    apt_install(['ca-certificates', 'curl'])

    codename = get_ubuntu_codename()
    setup_docker_apt_repo(codename, deb_arch)
    install_docker_packages()
    configure_docker_group()


def uninstall():
    """Uninstall Docker."""
    run_sudo_command([
        'apt-get', 'remove', '-y',
        'docker', 'docker-engine', 'docker.io',
        'docker-ce', 'docker-ce-cli', 'containerd.io',
        'docker-buildx-plugin', 'docker-compose-plugin',
    ], check=False)
    run_sudo_command(['apt-get', 'purge', '-y', 'docker-ce'], check=False)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Install or uninstall Docker on Ubuntu system'
    )
    parser.add_argument(
        'action',
        nargs='?',
        default='install',
        choices=['install', 'uninstall'],
        help='Action to perform (default: install)'
    )

    args = parser.parse_args()

    if args.action == 'install':
        install()
    elif args.action == 'uninstall':
        uninstall()


if __name__ == '__main__':
    main()
