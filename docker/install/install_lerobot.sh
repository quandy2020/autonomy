#!/usr/bin/env bash
# Install Hugging Face LeRobot for autonomy_lerobot (ROS2 uses /opt/venv python3).
set -e

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=installer_base.sh
. "${CURR_DIR}/installer_base.sh"

LEROBOT_VERSION="${LEROBOT_VERSION:-0.4.4}"
PY="$(python3_bin)"

info "Installing lerobot==${LEROBOT_VERSION} into ${PY} (may take several minutes)..."
"${PY}" -m pip install --timeout 120 --no-cache-dir "lerobot==${LEROBOT_VERSION}"

info "Pinning setuptools for colcon compatibility..."
pip3_install 'setuptools>=61,<80'

"${PY}" -c "import lerobot; from lerobot.datasets.lerobot_dataset import LeRobotDataset; print('lerobot', lerobot.__version__, 'ok')"

ok "lerobot ${LEROBOT_VERSION} installed"
