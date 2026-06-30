#!/usr/bin/env bash
# Prefer /opt/venv over Isaac Sim kit python (broken pip: packaging._structures).
set -euo pipefail

MARKER='# autonomy: prefer /opt/venv python over Isaac kit'
BASHRC=/etc/bash.bashrc

if [ -x /opt/venv/bin/python3 ] && ! grep -qF "${MARKER}" "${BASHRC}" 2>/dev/null; then
  {
    echo ''
    echo "${MARKER}"
    echo 'export PATH="/opt/venv/bin:${PATH}"'
    echo 'alias pip="/opt/venv/bin/python3 -m pip"'
    echo 'alias pip3="/opt/venv/bin/python3 -m pip"'
  } >> "${BASHRC}"
fi

if [ -x /opt/venv/bin/python3 ]; then
  ln -sf /opt/venv/bin/python3 /usr/local/bin/autonomy-python3
  cat > /usr/local/bin/autonomy-pip <<'EOF'
#!/bin/sh
exec /opt/venv/bin/python3 -m pip "$@"
EOF
  chmod +x /usr/local/bin/autonomy-pip
fi
