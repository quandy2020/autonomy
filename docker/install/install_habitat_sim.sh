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

# ------------------------------------------------------------------------------
# 安装 habitat-sim（与文档 02_Installation/prerequisite_software_installation_guide.md 一致）
# 方式：使用 Python 构建与安装，而非手写 CMake 构建。
# 默认使用 headless 模式，适用于 Docker / 服务器环境。
# ------------------------------------------------------------------------------

HABITAT_REPO_URL=${HABITAT_REPO_URL:-https://github.com/facebookresearch/habitat-sim.git}
HABITAT_BRANCH=${HABITAT_BRANCH:-stable}
HABITAT_ROOT=/thirdparty

echo "[habitat-sim] 使用仓库: ${HABITAT_REPO_URL}, 分支: ${HABITAT_BRANCH}"

# 获取源码
mkdir -p "${HABITAT_ROOT}"
cd "${HABITAT_ROOT}"

if [ ! -d habitat-sim ]; then
  git clone --branch "${HABITAT_BRANCH}" "${HABITAT_REPO_URL}" habitat-sim
fi

cd habitat-sim

# 初始化并更新子模块（与文档一致）
git submodule init
git submodule update

# 安装 Python 依赖
if [ -f requirements.txt ]; then
  echo "[habitat-sim] 安装 Python 依赖..."
  pip3 install -r requirements.txt
fi

# Docker/容器环境下的 Git 安全设置，避免权限问题
git config --global --add safe.directory '*'

echo "[habitat-sim] 开始 headless 模式安装（无图形界面）..."

# 与文档一致的缺省安装命令：
#   python3 setup.py install --headless --no-update-submodules
# 如需启用 CUDA / Bullet 等，可在外部通过环境变量追加到 EXTRA_HABITAT_BUILD_ARGS。
EXTRA_HABITAT_BUILD_ARGS=${EXTRA_HABITAT_BUILD_ARGS:-}

python3 setup.py install \
  --headless \
  --no-update-submodules \
  ${EXTRA_HABITAT_BUILD_ARGS}

echo "[habitat-sim] 安装完成。"