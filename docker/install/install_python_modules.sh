#!/usr/bin/env bash

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

set -e

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. "${CURR_DIR}/installer_base.sh"

if python3 -c 'import myst_parser, numpy; assert numpy.__version__ == "1.26.4"' 2>/dev/null; then
    ok "Python modules already installed, skipping"
    exit 0
fi

if ! command -v sphinx-build >/dev/null 2>&1; then
    apt_get_update_and_install python3-sphinx
fi

# Do not let pip uninstall Debian-managed Sphinx when mixing apt + pip on Ubuntu 24.04.
pip_args=(-r "${CURR_DIR}/py3_requirements.txt")
if pip_needs_break_system_packages "$(python3_bin)"; then
    pip_args+=(--ignore-installed sphinx)
fi

pip3_install "${pip_args[@]}"

if [[ "$(id -u)" -eq 0 ]]; then
    apt-get clean && rm -rf /var/lib/apt/lists/*
fi

ok "Successfully installed Python modules from py3_requirements.txt"
