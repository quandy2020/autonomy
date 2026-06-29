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
set -euo pipefail

# ROS nodes (habitat_node.py) use /opt/venv/bin/python3 — install into venv when present.
if [ -x /opt/venv/bin/python3 ]; then
  PY=/opt/venv/bin/python3
else
  PY=python3
fi
pip3() { "${PY}" -m pip "$@"; }

# Isaac Lab 等运行时镜像通常没有 CUDA toolkit；install_ccache.sh 还可能把 nvcc 链到 ccache。
detect_cuda_toolkit_root() {
  local candidate nvcc_path
  for candidate in "${CUDA_HOME:-}" "${CUDA_PATH:-}" /usr/local/cuda /usr/local/cuda-*; do
    [ -n "${candidate}" ] || continue
    [ -d "${candidate}/lib64" ] || continue
    nvcc_path="${candidate}/bin/nvcc"
    [ -x "${nvcc_path}" ] || continue
    if readlink -f "${nvcc_path}" 2>/dev/null | grep -q '/ccache$'; then
      continue
    fi
    if ! "${nvcc_path}" --version 2>&1 | grep -q 'Cuda compilation tools'; then
      continue
    fi
    echo "${candidate}"
    return 0
  done
  return 1
}

configure_habitat_cuda() {
  local mode="${HABITAT_WITH_CUDA:-auto}"
  local cuda_root=""

  if [ "${mode}" = "auto" ]; then
    if cuda_root="$(detect_cuda_toolkit_root)"; then
      mode=ON
    else
      mode=OFF
    fi
  elif [ "${mode}" = "ON" ]; then
    cuda_root="$(detect_cuda_toolkit_root)" || {
      echo "[habitat] HABITAT_WITH_CUDA=ON but CUDA toolkit not found; using CPU build."
      mode=OFF
    }
  fi

  export HABITAT_WITH_CUDA="${mode}"
  if [ "${mode}" = "ON" ]; then
    export CUDA_HOME="${cuda_root}"
    export CUDA_PATH="${cuda_root}"
    export PATH="${cuda_root}/bin:${PATH}"
    export LD_LIBRARY_PATH="${cuda_root}/lib64:${LD_LIBRARY_PATH:-}"
    EXTRA_HABITAT_BUILD_ARGS="${EXTRA_HABITAT_BUILD_ARGS} --config-settings=cmake.define.CUDA_TOOLKIT_ROOT_DIR=${cuda_root}"
    echo "[habitat] CUDA build enabled (toolkit: ${cuda_root})"
  else
    echo "[habitat] CPU-only build (HABITAT_WITH_CUDA=OFF)"
  fi
}
# 安装 habitat-sim + habitat-lab v0.3.4（版本需保持一致）
# habitat-sim：pip install 从源码编译，适用于 Docker / 服务器 headless 环境。
# habitat-lab：pip install -e habitat-lab/
# 环境变量示例：
#   HABITAT_WITH_CUDA=auto|ON|OFF  auto=检测到 toolkit 才启用 CUDA（默认）
#   HABITAT_WITH_BULLET=OFF     禁用 Bullet 物理引擎
#   INSTALL_HABITAT_BASELINES=1 可选安装 habitat-baselines
# ------------------------------------------------------------------------------

HABITAT_REPO_URL=${HABITAT_REPO_URL:-https://github.com/facebookresearch/habitat-sim.git}
HABITAT_VERSION=${HABITAT_VERSION:-v0.3.4}
HABITAT_LAB_REPO_URL=${HABITAT_LAB_REPO_URL:-https://github.com/facebookresearch/habitat-lab.git}
HABITAT_LAB_VERSION=${HABITAT_LAB_VERSION:-v0.3.4}
HABITAT_ROOT=/thirdparty

echo "[habitat] Using Python: ${PY} ($("${PY}" --version))"
echo "[habitat-sim] 使用仓库: ${HABITAT_REPO_URL}, 版本: ${HABITAT_VERSION}"

mkdir -p "${HABITAT_ROOT}"
cd "${HABITAT_ROOT}"

if [ ! -d habitat-sim/.git ]; then
  rm -rf habitat-sim
  git clone --branch "${HABITAT_VERSION}" "${HABITAT_REPO_URL}" habitat-sim
else
  cd habitat-sim
  git fetch --tags origin
  git checkout "${HABITAT_VERSION}"
  cd ..
fi

cd habitat-sim
git submodule update --init --recursive

# habitat-lab 固定 numpy==1.26.4；habitat-sim 元数据要求 numpy>=2.0 但运行时兼容 1.26.4
if [ -f requirements.txt ]; then
  echo "[habitat-sim] 调整 numpy 版本约束以兼容 habitat-lab..."
  sed -i 's/numpy>=2.0.0,<2.4/numpy==1.26.4/' requirements.txt
fi
if [ -f pyproject.toml ]; then
  sed -i 's/"numpy>=2.0.0,<2.4"/"numpy==1.26.4"/' pyproject.toml
fi

pip3 install --no-cache-dir "numpy==1.26.4"

if [ -f requirements.txt ]; then
  echo "[habitat-sim] 安装 Python 依赖..."
  pip3 install --no-cache-dir -r requirements.txt
fi

echo "[habitat-sim] 安装构建依赖 (scikit-build-core, pybind11)..."
pip3 install --no-cache-dir "scikit-build-core>=0.10" "pybind11>=2.10"

git config --global --add safe.directory '*'

echo "[habitat-sim] 开始 headless 模式安装（无图形界面）..."
export HABITAT_BUILD_GUI_VIEWERS=${HABITAT_BUILD_GUI_VIEWERS:-OFF}
EXTRA_HABITAT_BUILD_ARGS=${EXTRA_HABITAT_BUILD_ARGS:-}
configure_habitat_cuda
rm -rf build

pip3 install . \
  --no-build-isolation \
  ${EXTRA_HABITAT_BUILD_ARGS}

echo "[habitat-sim] 安装完成。"

# ------------------------------------------------------------------------------
# 安装 habitat-lab（依赖已安装的 habitat-sim）
# ------------------------------------------------------------------------------

echo "[habitat-lab] 使用仓库: ${HABITAT_LAB_REPO_URL}, 版本: ${HABITAT_LAB_VERSION}"

cd "${HABITAT_ROOT}"

if [ ! -d habitat-lab/.git ]; then
  rm -rf habitat-lab
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

echo "[habitat-lab] 锁定 numpy==1.26.4..."
pip3 install --no-cache-dir "numpy==1.26.4"

# Docker 镜像中 install_opencv.sh 编译的 OpenCV 5.x 可能污染 pip cv2 包
echo "[habitat-lab] 修复 OpenCV 冲突..."
PYTHON_SITE="$("${PY}" -c 'import site; print(site.getsitepackages()[0])')"
pip3 uninstall -y opencv-python 2>/dev/null || true
rm -rf "${PYTHON_SITE}/cv2" "${PYTHON_SITE}"/opencv_python*.dist-info
pip3 install --no-cache-dir "opencv-python==4.11.0.86"

echo "[habitat-lab] 安装 HuNav/ROS Python 依赖 (openai, rich)..."
pip3 install --no-cache-dir openai rich

if [ "${INSTALL_HABITAT_BASELINES:-0}" = "1" ]; then
  echo "[habitat-lab] 安装 habitat-baselines..."
  pip3 install -e habitat-baselines/ ${EXTRA_HABITAT_LAB_BUILD_ARGS}
fi

echo "[habitat-lab] 验证安装..."
"${PY}" -c "import habitat_sim; import habitat; print('habitat-sim', habitat_sim.__version__)"
echo "[habitat-lab] 安装完成。"
