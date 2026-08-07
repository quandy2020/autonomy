#!/usr/bin/env bash
# Build standalone autolink Python bindings (required by autoviz Python examples).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)"
SRC="${ROOT}/src/autonomy/autolink"
BUILD="${ROOT}/build/autolink-python"

# Avoid Isaac Sim / mixed-venv Python breaking cmake (SRE module mismatch).
export PATH="/usr/bin:/bin:/usr/local/bin:${PATH}"
unset PYTHONHOME PYTHONPATH VIRTUAL_ENV CONDA_PREFIX
PYTHON="${AUTOVIZ_PYTHON:-/usr/bin/python3}"
if [[ ! -x "${PYTHON}" ]]; then
  PYTHON="/usr/bin/python3.10"
fi

echo "Repo:   ${ROOT}"
echo "Build:  ${BUILD}"
echo "Python: ${PYTHON}"

cmake -S "${SRC}" -B "${BUILD}" \
  -DAUTOLINK_BUILD_PYTHON=ON \
  -DAUTOLINK_BUILD_TEST=OFF \
  -DPython3_EXECUTABLE="${PYTHON}"
cmake --build "${BUILD}" -j"$(nproc)"

echo
echo "Done. Run the tutorial with system Python:"
echo "  cd ${ROOT}/src/autonomy/autoviz"
echo "  /usr/bin/python3 examples/python/01_tutorial_show_path.py"
