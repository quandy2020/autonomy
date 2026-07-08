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

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

gperftools_lib_present() {
    [[ -f /usr/lib/libtcmalloc.so ]] \
        || [[ -f /usr/lib/x86_64-linux-gnu/libtcmalloc.so ]] \
        || [[ -f /usr/lib/aarch64-linux-gnu/libtcmalloc.so ]] \
        || [[ -f /usr/local/lib/libtcmalloc.so ]]
}

if gperftools_lib_present; then
    ok "gperftools already installed, skipping source build"
    exit 0
fi

apt_get_update_and_install \
    libunwind8 \
    libunwind-dev \
    graphviz

VERSION="2.8"
PKG_NAME="gperftools-${VERSION}.tar.gz"
CHECKSUM="b09193adedcc679df2387042324d0d54b93d35d062ea9bff0340f342a709e860"
DOWNLOAD_LINK="https://github.com/gperftools/gperftools/archive/${PKG_NAME}"

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf ${PKG_NAME}

pushd "gperftools-gperftools-${VERSION}" >/dev/null
    ./autogen.sh || sleep 1 && ./autogen.sh
    ./configure --prefix=/usr
    # shared lib only options: --enable-static=no --with-pic=yes
    make -j$(nproc)
    make install
popd >/dev/null

ldconfig

ok "Successfully installed gperftools-${VERSION}."

# Keep libunwind-dev installed: purging it can autoremove glog and other deps.
rm -rf ${PKG_NAME} "gperftools-gperftools-${VERSION}"
