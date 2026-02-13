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

###############################################################################
# 安装 GUI 相关依赖，使 OpenCV 在 Docker 中支持窗口显示 (GTK highgui)
###############################################################################
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    pkg-config \
    libgtk-3-dev \
    libcanberra-gtk3-module \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev

###############################################################################
# 获取 OpenCV 源码
###############################################################################
cd /thirdparty
git clone -b 5.x https://github.com/opencv/opencv.git
git clone -b 5.x https://github.com/opencv/opencv_contrib.git

###############################################################################
# 配置并编译 OpenCV（启用 GTK 高级 GUI）
###############################################################################
cd opencv && mkdir build && cd build
cmake  \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
    -DWITH_GTK=ON \
    -DWITH_QT=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    ..

# build
make -j"$(nproc)"
sudo make install
sudo ldconfig

###############################################################################
# 清理中间文件，减小镜像体积
###############################################################################
cd .. && rm -rf build
