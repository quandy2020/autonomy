#!/usr/bin/env bash
# Install Hugging Face LeRobot into /opt/venv (ROS2 / habitat use the same interpreter).
set -euo pipefail

INSTALLERS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=installer_base.sh
. "${INSTALLERS_DIR}/installer_base.sh"

bash "${INSTALLERS_DIR}/setup_autonomy_python.sh" 2>/dev/null || true

LEROBOT_VERSION="${LEROBOT_VERSION:-0.4.4}"
PY="$(python3_bin)"

if "${PY}" -c "import lerobot" 2>/dev/null; then
  ok "lerobot already installed: $("${PY}" -c 'import lerobot; print(lerobot.__version__)')"
  exit 0
fi

info "Installing lerobot==${LEROBOT_VERSION} into ${PY} (may take several minutes)..."
"${PY}" -m pip install --timeout 120 --no-cache-dir "lerobot==${LEROBOT_VERSION}"

info "Pinning setuptools for colcon compatibility..."
pip3_install 'setuptools>=61,<80'

"${PY}" -c "import lerobot; from lerobot.datasets.lerobot_dataset import LeRobotDataset; print('lerobot', lerobot.__version__, 'ok')"

ok "lerobot ${LEROBOT_VERSION} installed"
