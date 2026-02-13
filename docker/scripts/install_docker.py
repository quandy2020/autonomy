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
import subprocess
import sys
from pathlib import Path


def run_command(cmd, check=True, shell=False):
    """Run a shell command and return the result."""
    if isinstance(cmd, str) and not shell:
        cmd = cmd.split()
    result = subprocess.run(
        cmd,
        shell=shell,
        check=check,
        capture_output=True,
        text=True
    )
    return result


def run_sudo_command(cmd, check=True, shell=False):
    """Run a command with sudo."""
    if isinstance(cmd, str) and not shell:
        cmd = cmd.split()
    sudo_cmd = ['sudo'] + (cmd if isinstance(cmd, list) else [cmd])
    return run_command(sudo_cmd, check=check, shell=shell)


def install_filesystem_support():
    """Install filesystem support for Docker (overlay or aufs)."""
    # Get kernel version
    kernel_version = platform.release()
    machine_version = kernel_version
    
    # Get main kernel version (first character)
    main_kernel_version = int(kernel_version[0]) if kernel_version[0].isdigit() else 0
    
    overlay_module_path = Path(f'/lib/modules/{machine_version}/kernel/fs/overlayfs/overlay.ko')
    
    if main_kernel_version > 3 and overlay_module_path.exists():
        print("the kernel version 4 or higher;")
        print("it has support overlay2")
        run_sudo_command(['modprobe', 'overlay'])
    else:
        print("the kernel version is lower than 4")
        print("try to install aufs")
        run_sudo_command(['apt-get', 'update'])
        run_sudo_command([
            'apt-get', 'install', '-y',
            f'linux-image-extra-{machine_version}',
            'linux-image-extra-virtual'
        ])
    
    # Install prerequisites
    run_sudo_command(['apt-get', 'update'])
    run_sudo_command([
        'apt-get', 'install', '-y',
        'apt-transport-https',
        'ca-certificates',
        'curl',
        'software-properties-common'
    ])
    
    # Add Docker GPG key
    curl_process = subprocess.Popen(
        ['curl', '-fsSL', 'https://download.docker.com/linux/ubuntu/gpg'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    apt_key_process = subprocess.Popen(
        ['sudo', 'apt-key', 'add', '-'],
        stdin=curl_process.stdout,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    curl_process.stdout.close()
    apt_key_output, apt_key_error = apt_key_process.communicate()
    if apt_key_process.returncode != 0:
        # Fallback: use apt-key directly
        run_sudo_command([
            'apt-key', 'adv',
            '--keyserver', 'hkp://keyserver.ubuntu.com:80',
            '--recv-keys', '0EBFCD88'
        ], check=False)


def install_docker_x86():
    """Install Docker on x86_64 architecture."""
    # Get Ubuntu codename
    try:
        result = run_command(['lsb_release', '-cs'], check=True)
        ubuntu_codename = result.stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("Error: lsb_release not found. Cannot determine Ubuntu codename.")
        sys.exit(1)
    
    # Add Docker repository
    repo = f"deb [arch=amd64] https://download.docker.com/linux/ubuntu {ubuntu_codename} stable"
    run_sudo_command([
        'add-apt-repository', '-y', repo
    ])
    
    # Update and install docker-ce
    run_sudo_command(['apt-get', 'update'])
    run_sudo_command(['apt-get', 'install', '-y', 'docker-ce'])
    
    # Create docker group and add user
    result = run_sudo_command(['groupadd', 'docker'], check=False)
    # Group might already exist, which is fine
    
    current_user = os.getenv('USER') or os.getenv('USERNAME')
    if current_user:
        run_sudo_command(['gpasswd', '-a', current_user, 'docker'])
        print(f"\nUser {current_user} has been added to the docker group.")
        print("Please log out and log back in for the changes to take effect.")
        print("Alternatively, you can run: newgrp docker")


def install():
    """Main installation function."""
    # Install filesystem support
    install_filesystem_support()
    
    # Check machine architecture
    machine_arch = platform.machine()
    if machine_arch == 'x86_64':
        install_docker_x86()
    else:
        print(f"Unknown machine architecture {machine_arch}")
        sys.exit(1)


def uninstall():
    """Uninstall Docker."""
    run_sudo_command([
        'apt-get', 'remove', '-y',
        'docker', 'docker-engine', 'docker.io'
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

