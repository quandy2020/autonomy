#!/usr/bin/env bash
# Install habitat-sim + habitat-lab into /opt/venv if missing.
set -euo pipefail

INSTALLERS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

if [ -x /opt/venv/bin/python3 ]; then
  PY=/opt/venv/bin/python3
else
  PY=python3
fi

if "${PY}" -c "import habitat_sim" 2>/dev/null; then
  echo "habitat-sim already installed: $("${PY}" -c 'import habitat_sim; print(habitat_sim.__version__)')"
  exit 0
fi

echo "habitat-sim not found in ${PY}. Installing (may take 30-60 minutes)..."
HABITAT_WITH_CUDA=${HABITAT_WITH_CUDA:-auto} bash "${INSTALLERS_DIR}/install_habitat.sh"
