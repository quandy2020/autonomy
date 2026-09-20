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

THIRDPARTY="$(autonomy_thirdparty_dir)"
INSTALL_PREFIX="$(autonomy_cmake_install_prefix)"
THREAD_NUM=$(nproc)

# Headless board: prefer GTK when installable; otherwise build without highgui GTK.
WITH_GTK=OFF
if dpkg -s libgtk-3-dev >/dev/null 2>&1; then
    WITH_GTK=ON
else
    set +e
    apt_get_update_and_install libgtk-3-dev libcanberra-gtk3-module
    gtk_rc=$?
    set -e
    if [[ "${gtk_rc}" -eq 0 ]]; then
        WITH_GTK=ON
    fi
fi

if [[ -f "${INSTALL_PREFIX}/lib/libopencv_core.so" ]]; then
    ok "OpenCV already installed under ${INSTALL_PREFIX}, skipping"
    exit 0
fi

info "Installing OpenCV -> ${INSTALL_PREFIX} (GTK=${WITH_GTK})"

apt_get_update_and_install \
    build-essential \
    cmake \
    git \
    pkg-config \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev || true

cd "${THIRDPARTY}"
if [[ ! -d opencv/.git ]]; then
    rm -rf opencv
    git_clone_with_retry https://github.com/opencv/opencv.git 4.10.0 opencv
fi
if [[ ! -d opencv_contrib/.git ]]; then
    rm -rf opencv_contrib
    git_clone_with_retry https://github.com/opencv/opencv_contrib.git 4.10.0 opencv_contrib
fi

cd opencv
rm -rf build
mkdir build && cd build
cmake \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DOPENCV_EXTRA_MODULES_PATH="${THIRDPARTY}/opencv_contrib/modules" \
    -DWITH_GTK="${WITH_GTK}" \
    -DWITH_QT=OFF \
    -DWITH_HDF5=OFF \
    -DBUILD_opencv_hdf=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_opencv_python=OFF \
    -DBUILD_EXAMPLES=OFF \
    ..

make -j"${THREAD_NUM}"
make install
ldconfig

ok "Successfully installed OpenCV -> ${INSTALL_PREFIX}"
