#!/usr/bin/env bash
# One-click install Intel librealsense2 for autodriver (D455 / RGB-D + IMU).
# Upstream: https://github.com/IntelRealSense/librealsense
#
# Default: Intel apt repository (librealsense2 + -dev + -utils).
# Fallback / alternate: build from source and install to PREFIX.
#
# Usage:
#   ./scripts/install_realsense_sdk.sh
#   REALSENSE_SDK_METHOD=source ./scripts/install_realsense_sdk.sh
#   REALSENSE_SDK_VERSION=v2.55.1 ./scripts/install_realsense_sdk.sh   # source tag
#   ./scripts/install_realsense_sdk.sh /path/to/librealsense           # source tree
#
# Env:
#   PREFIX                  install prefix for source builds (default: /usr/local)
#   REALSENSE_SDK_METHOD    apt | source (default: apt)
#   REALSENSE_SDK_VERSION   git tag/branch for source (default: latest release tag)
#   REALSENSE_SDK_DIR       clone/cache dir (default: /tmp/librealsense-src)
#   REALSENSE_SDK_REPO      git URL
#   REALSENSE_SKIP_UDEV     set to 1 to skip udev rules (source method)
#   REALSENSE_FORCE         set to 1 to reinstall even if already present
#   REALSENSE_WITH_EXAMPLES set to 1 to build SDK examples (source; default off)

set -euo pipefail

REPO_URL="${REALSENSE_SDK_REPO:-https://github.com/IntelRealSense/librealsense.git}"
REPO_API="${REALSENSE_SDK_REPO_API:-https://api.github.com/repos/IntelRealSense/librealsense/releases}"
METHOD="${REALSENSE_SDK_METHOD:-apt}"
PREFIX="${PREFIX:-/usr/local}"
CACHE_DIR="${REALSENSE_SDK_DIR:-/tmp/librealsense-src}"
APT_KEYRING="/etc/apt/keyrings/librealsenseai.gpg"
APT_LIST="/etc/apt/sources.list.d/librealsense.list"
APT_ASC_URL="https://librealsense.realsenseai.com/Debian/librealsenseai.asc"
APT_REPO_URL="https://librealsense.realsenseai.com/Debian/apt-repo"

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "missing required command: $1" >&2
    exit 1
  }
}

run_priv() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  else
    need_cmd sudo
    sudo "$@"
  fi
}

sdk_already_installed() {
  if [[ "${REALSENSE_FORCE:-0}" == "1" ]]; then
    return 1
  fi
  if pkg-config --exists realsense2 2>/dev/null; then
    return 0
  fi
  if [[ -f "${PREFIX}/include/librealsense2/rs.hpp" ]] \
    || [[ -f /usr/include/librealsense2/rs.hpp ]] \
    || [[ -f "${PREFIX}/lib/librealsense2.so" ]] \
    || [[ -f /usr/lib/x86_64-linux-gnu/librealsense2.so ]] \
    || [[ -f /usr/lib/aarch64-linux-gnu/librealsense2.so ]]; then
    return 0
  fi
  if command -v dpkg >/dev/null 2>&1 && dpkg -s librealsense2-dev >/dev/null 2>&1; then
    return 0
  fi
  return 1
}

print_version() {
  if pkg-config --exists realsense2 2>/dev/null; then
    echo "librealsense2: $(pkg-config --modversion realsense2)"
  fi
  if command -v rs-enumerate-devices >/dev/null 2>&1; then
    echo "rs-enumerate-devices: $(command -v rs-enumerate-devices)"
    rs-enumerate-devices -s 2>/dev/null || true
  fi
}

print_done() {
  echo
  echo "librealsense2 installed."
  print_version
  echo "Rebuild autodriver with:"
  echo "  cmake -S . -B build -DAUTODRIVER_WITH_REALSENSE=ON"
  echo "  cmake --build build -j\"\$(nproc)\" --target autodriver"
  echo "Confirm configure status: autodriver: librealsense2 … enabled"
  echo "Smoke: ./scripts/verify_realsense_d455.sh"
}

install_udev_from_tree() {
  local root="$1"
  local rules=""
  for cand in \
    "${root}/config/99-realsense-libusb.rules" \
    "${root}/scripts/udev-rules/99-realsense-libusb.rules"; do
    if [[ -f "${cand}" ]]; then
      rules="${cand}"
      break
    fi
  done
  if [[ -z "${rules}" ]]; then
    echo "udev rules not found under ${root}; skip" >&2
    return 0
  fi
  echo "== Installing RealSense udev rules =="
  run_priv cp -f "${rules}" /etc/udev/rules.d/99-realsense-libusb.rules
  if command -v udevadm >/dev/null 2>&1; then
    run_priv udevadm control --reload-rules || true
    run_priv udevadm trigger || true
  fi
}

install_via_apt() {
  need_cmd curl
  need_cmd apt-get
  need_cmd gpg

  if [[ ! -f /etc/os-release ]]; then
    echo "apt method requires Debian/Ubuntu (/etc/os-release missing)" >&2
    exit 1
  fi
  # shellcheck disable=SC1091
  . /etc/os-release
  local codename="${VERSION_CODENAME:-}"
  if [[ -z "${codename}" ]]; then
    echo "cannot detect VERSION_CODENAME; use REALSENSE_SDK_METHOD=source" >&2
    exit 1
  fi

  echo "== Installing librealsense2 from Intel apt (${codename}) =="
  run_priv apt-get update
  run_priv DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg apt-transport-https lsb-release usbutils

  run_priv install -m 0755 -d /etc/apt/keyrings
  curl -fsSL "${APT_ASC_URL}" \
    | run_priv gpg --dearmor -o "${APT_KEYRING}"
  run_priv chmod a+r "${APT_KEYRING}"

  echo "deb [signed-by=${APT_KEYRING}] ${APT_REPO_URL} ${codename} main" \
    | run_priv tee "${APT_LIST}" >/dev/null

  run_priv apt-get update
  run_priv DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    librealsense2 \
    librealsense2-dev \
    librealsense2-utils

  if command -v ldconfig >/dev/null 2>&1; then
    run_priv ldconfig || true
  fi
}

resolve_release_tag() {
  if [[ -n "${REALSENSE_SDK_VERSION:-}" ]]; then
    echo "${REALSENSE_SDK_VERSION}"
    return
  fi
  need_cmd curl
  need_cmd python3
  local json
  json="$(curl -fsSL -A 'autodriver-install-realsense' "${REPO_API}/latest")"
  python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])' <<<"${json}"
}

download_source() {
  need_cmd git
  local ref
  ref="$(resolve_release_tag)"
  mkdir -p "$(dirname "${CACHE_DIR}")"
  if [[ -d "${CACHE_DIR}/.git" ]]; then
    echo "== Updating ${CACHE_DIR} (${ref}) =="
    git -C "${CACHE_DIR}" fetch --depth 1 origin "refs/tags/${ref}:refs/tags/${ref}" \
      || git -C "${CACHE_DIR}" fetch --depth 1 origin "${ref}" || true
    git -C "${CACHE_DIR}" checkout -q "${ref}" || \
      git -C "${CACHE_DIR}" checkout -q "FETCH_HEAD"
  else
    echo "== Cloning ${REPO_URL} (${ref}) -> ${CACHE_DIR} =="
    rm -rf "${CACHE_DIR}"
    if ! git clone --depth 1 --branch "${ref}" "${REPO_URL}" "${CACHE_DIR}"; then
      git clone --depth 1 "${REPO_URL}" "${CACHE_DIR}"
      git -C "${CACHE_DIR}" fetch --depth 1 origin "refs/tags/${ref}:refs/tags/${ref}" || true
      git -C "${CACHE_DIR}" checkout -q "${ref}" || true
    fi
  fi
  echo "${CACHE_DIR}"
}

install_from_source() {
  local src="$1"
  need_cmd cmake
  need_cmd make
  need_cmd g++
  if [[ ! -f "${src}/CMakeLists.txt" ]] || [[ ! -d "${src}/include/librealsense2" ]]; then
    echo "librealsense source invalid: ${src}" >&2
    exit 1
  fi

  # Build deps commonly needed on Ubuntu.
  if command -v apt-get >/dev/null 2>&1; then
    run_priv apt-get update
    run_priv DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      git cmake build-essential pkg-config libusb-1.0-0-dev libssl-dev \
      libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev || true
  fi

  local build_dir="${src}/build-autodriver"
  local examples=OFF
  if [[ "${REALSENSE_WITH_EXAMPLES:-0}" == "1" ]]; then
    examples=ON
  fi

  echo "== Building librealsense in ${build_dir} =="
  # FORCE_RSUSB_BACKEND helps Jetson / boards without dkms kernel patches.
  cmake -S "${src}" -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_EXAMPLES="${examples}" \
    -DBUILD_GRAPHICAL_EXAMPLES="${examples}" \
    -DBUILD_TOOLS=ON \
    -DFORCE_RSUSB_BACKEND=ON
  cmake --build "${build_dir}" -j"$(nproc)"

  echo "== Installing to ${PREFIX} =="
  if [[ -w "${PREFIX}" ]] || [[ "$(id -u)" -eq 0 ]]; then
    cmake --install "${build_dir}" --prefix "${PREFIX}"
  else
    run_priv cmake --install "${build_dir}" --prefix "${PREFIX}"
  fi

  if [[ "${REALSENSE_SKIP_UDEV:-0}" != "1" ]]; then
    install_udev_from_tree "${src}"
  fi

  if command -v ldconfig >/dev/null 2>&1; then
    run_priv ldconfig || true
  fi
}

# --- main ---
if sdk_already_installed; then
  echo "librealsense2 already present (set REALSENSE_FORCE=1 to reinstall)."
  print_done
  exit 0
fi

ARG="${1:-}"

if [[ -n "${ARG}" && -d "${ARG}" && -f "${ARG}/CMakeLists.txt" ]]; then
  install_from_source "${ARG}"
  print_done
  exit 0
fi

case "${METHOD}" in
  apt)
    if ! install_via_apt; then
      echo "apt install failed; retry with REALSENSE_SDK_METHOD=source" >&2
      exit 1
    fi
    ;;
  source)
    SRC="$(download_source)"
    install_from_source "${SRC}"
    ;;
  *)
    echo "unknown REALSENSE_SDK_METHOD=${METHOD} (use apt|source)" >&2
    exit 1
    ;;
esac

print_done
