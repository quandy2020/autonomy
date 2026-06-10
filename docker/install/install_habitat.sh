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
# 安装 habitat-sim + habitat-lab v0.3.4（最新稳定版，版本需保持一致）
# habitat-sim：pip install 从源码编译，适用于 Docker / 服务器 headless 环境。
# habitat-lab：pip install -e habitat-lab/（参见 habitat-lab/Dockerfile）
# habitat-sim 构建选项通过环境变量控制，例如：
#   HABITAT_WITH_CUDA=ON        启用 CUDA
#   HABITAT_WITH_BULLET=OFF     禁用 Bullet 物理引擎（默认启用）
#   HABITAT_WITH_AUDIO=ON       启用音频传感器（仅 Linux）
# 额外 pip 参数可通过 EXTRA_HABITAT_BUILD_ARGS 传入，例如：
#   --config-settings=cmake.define.CMAKE_BUILD_PARALLEL_LEVEL=1
# 可选安装 habitat-baselines：INSTALL_HABITAT_BASELINES=1
# ------------------------------------------------------------------------------

HABITAT_REPO_URL=${HABITAT_REPO_URL:-https://github.com/facebookresearch/habitat-sim.git}
HABITAT_VERSION=${HABITAT_VERSION:-v0.3.4}
HABITAT_LAB_REPO_URL=${HABITAT_LAB_REPO_URL:-https://github.com/facebookresearch/habitat-lab.git}
HABITAT_LAB_VERSION=${HABITAT_LAB_VERSION:-v0.3.4}
HABITAT_ROOT=/thirdparty

echo "[habitat-sim] 使用仓库: ${HABITAT_REPO_URL}, 版本: ${HABITAT_VERSION}"

# 获取源码
mkdir -p "${HABITAT_ROOT}"
cd "${HABITAT_ROOT}"

if [ ! -d habitat-sim ]; then
  git clone --branch "${HABITAT_VERSION}" "${HABITAT_REPO_URL}" habitat-sim
else
  cd habitat-sim
  git fetch --tags origin
  git checkout "${HABITAT_VERSION}"
  cd ..
fi

cd habitat-sim

# 初始化并更新子模块
git submodule init
git submodule update

# habitat-lab 固定 numpy==1.26.4；habitat-sim 元数据要求 numpy>=2.0 但运行时兼容 1.26.4
# 统一使用 numpy 1.26.4，避免 habitat-lab 安装时降级引发冲突
if [ -f requirements.txt ]; then
  echo "[habitat-sim] 调整 numpy 版本约束以兼容 habitat-lab..."
  sed -i 's/numpy>=2.0.0,<2.4/numpy==1.26.4/' requirements.txt
fi
if [ -f pyproject.toml ]; then
  sed -i 's/"numpy>=2.0.0,<2.4"/"numpy==1.26.4"/' pyproject.toml
fi

# 安装 Python 依赖
if [ -f requirements.txt ]; then
  echo "[habitat-sim] 安装 Python 依赖..."
  pip3 install -r requirements.txt
fi

# habitat-sim v0.3.4+ 使用 scikit-build-core；--no-build-isolation 需预先安装构建后端
echo "[habitat-sim] 安装构建依赖 (scikit-build-core, pybind11)..."
pip3 install "scikit-build-core>=0.10" "pybind11>=2.10"

# Docker/容器环境下的 Git 安全设置，避免权限问题
git config --global --add safe.directory '*'

echo "[habitat-sim] 开始 headless 模式安装（无图形界面）..."

# v0.3.4 起使用 pip + 环境变量替代 setup.py --headless
export HABITAT_BUILD_GUI_VIEWERS=${HABITAT_BUILD_GUI_VIEWERS:-OFF}
EXTRA_HABITAT_BUILD_ARGS=${EXTRA_HABITAT_BUILD_ARGS:-}

pip3 install . \
  --no-build-isolation \
  ${EXTRA_HABITAT_BUILD_ARGS}

echo "[habitat-sim] 安装完成。"

# ------------------------------------------------------------------------------
# 安装 habitat-lab（依赖已安装的 habitat-sim）
# ------------------------------------------------------------------------------

echo "[habitat-lab] 使用仓库: ${HABITAT_LAB_REPO_URL}, 版本: ${HABITAT_LAB_VERSION}"

cd "${HABITAT_ROOT}"

if [ ! -d habitat-lab ]; then
  git clone --branch "${HABITAT_LAB_VERSION}" "${HABITAT_LAB_REPO_URL}" habitat-lab
else
  cd habitat-lab
  git fetch --tags origin
  git checkout "${HABITAT_LAB_VERSION}"
  cd ..
fi

cd habitat-lab

echo "[habitat-lab] 安装核心包..."
EXTRA_HABITAT_LAB_BUILD_ARGS=${EXTRA_HABITAT_LAB_BUILD_ARGS:-}
pip3 install -e habitat-lab/ ${EXTRA_HABITAT_LAB_BUILD_ARGS}

# 统一锁定 numpy，消除 habitat-sim 与 habitat-lab 之间的版本冲突
echo "[habitat-lab] 锁定 numpy==1.26.4..."
pip3 install "numpy==1.26.4"

# Docker 镜像中 install_opencv.sh 编译的 OpenCV 5.x 可能污染 pip cv2 包，
# 导致 habitat-lab 导入时 cv2.applyColorMap 失败；强制重装 opencv-python 4.x
echo "[habitat-lab] 修复 OpenCV 冲突..."
PYTHON_SITE="$(python3 -c 'import site; print(site.getsitepackages()[0])')"
pip3 uninstall -y opencv-python 2>/dev/null || true
rm -rf "${PYTHON_SITE}/cv2" "${PYTHON_SITE}"/opencv_python*.dist-info
pip3 install --no-cache-dir "opencv-python==4.11.0.86"

if [ "${INSTALL_HABITAT_BASELINES:-0}" = "1" ]; then
  echo "[habitat-lab] 安装 habitat-baselines..."
  pip3 install -e habitat-baselines/ ${EXTRA_HABITAT_LAB_BUILD_ARGS}
fi

echo "[habitat-lab] 安装完成。"
