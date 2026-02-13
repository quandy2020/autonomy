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

# Fail on first error.
set -e

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

# Install C++ dependencies: lz4 and zstd development libraries
info "Installing C++ dependencies for mcap..."
apt_get_update_and_install \
    liblz4-dev \
    libzstd-dev

# Install Python dependencies: lz4 and zstandard (for Python package)
info "Installing Python dependencies for mcap..."
pip3_install \
    lz4 \
    zstandard

cd /thirdparty

# Clone mcap repository
info "Cloning mcap repository..."
git clone https://github.com/foxglove/mcap.git
MCAP_DIR="/thirdparty/mcap"
cd ${MCAP_DIR} && git submodule init && git submodule update

# Install C++ version (header-only library)
info "Installing mcap C++ headers..."
if [ -d "${MCAP_DIR}/cpp/mcap/include/mcap" ]; then
    mkdir -p /usr/local/include
    cp -r ${MCAP_DIR}/cpp/mcap/include/mcap /usr/local/include/
    info "C++ headers installed to /usr/local/include/mcap"
else
    error "C++ headers not found in ${MCAP_DIR}/cpp/mcap/include/mcap"
    exit 1
fi

# Install Python version
info "Installing mcap Python package..."
cd ${MCAP_DIR}/python/mcap
pip3_install -e .
info "Python mcap package installed"

# Install optional Python support packages (optional, comment out if not needed)
# info "Installing mcap Python support packages..."
# cd ../mcap-protobuf-support
# pip3_install -e .
# cd ../mcap-ros1-support
# pip3_install -e .
# cd ../mcap-ros2-support
# pip3_install -e .

# Clean up
cd /thirdparty
rm -rf mcap

ok "Successfully installed mcap (C++ headers and Python package)."
