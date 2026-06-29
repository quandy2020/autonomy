#!/usr/bin/env bash
# Install Python packages required by ROS 2 nodes in /opt/venv.
set -euo pipefail

INSTALLERS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

python_bin() {
  if [ -x /opt/venv/bin/python3 ]; then
    echo /opt/venv/bin/python3
  else
    echo python3
  fi
}

PY="$(python_bin)"

if ! "${PY}" -c 'import numpy' 2>/dev/null; then
  echo "Installing numpy for ROS 2 Python nodes..."
  "${PY}" -m pip install --no-cache-dir numpy==1.26.4
else
  echo "numpy already installed: $("${PY}" -c 'import numpy; print(numpy.__version__)')"
fi

# Other colcon/rosidl build deps (idempotent).
"${PY}" -m pip install --no-cache-dir empy==3.3.4 lark catkin_pkg openai rich
