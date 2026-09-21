#!/usr/bin/env bash
# One-click install Orbbec SDK v2 for autodriver (Gemini 330 / RGB-D).
# Upstream: https://github.com/orbbec/OrbbecSDK_v2
#
# Default: download the official .deb for this arch and install via dpkg
# (headers + libs under /usr/local; CMake package OrbbecSDK).
#
# Usage:
#   ./scripts/install_orbbec_sdk.sh
#   ORBBEC_SDK_VERSION=v2.9.3 ./scripts/install_orbbec_sdk.sh
#   ORBBEC_SDK_METHOD=source ./scripts/install_orbbec_sdk.sh   # build from git
#   ./scripts/install_orbbec_sdk.sh /path/to/OrbbecSDK_v2.x.x_amd64.deb
#   ./scripts/install_orbbec_sdk.sh /path/to/OrbbecSDK_v2-src   # source tree
#
# Env:
#   PREFIX               install prefix for source builds (default: /usr/local)
#   ORBBEC_SDK_VERSION   release tag (default: latest GitHub release)
#   ORBBEC_SDK_METHOD    deb | source (default: deb)
#   ORBBEC_SDK_DIR       download/cache dir (default: /tmp/orbbec-sdk)
#   ORBBEC_SDK_REPO      git URL for source method
#   ORBBEC_SKIP_UDEV     set to 1 to skip udev rules
#   ORBBEC_FORCE         set to 1 to reinstall even if already present

set -euo pipefail

REPO_API="${ORBBEC_SDK_REPO_API:-https://api.github.com/repos/orbbec/OrbbecSDK_v2/releases}"
REPO_URL="${ORBBEC_SDK_REPO:-https://github.com/orbbec/OrbbecSDK_v2.git}"
METHOD="${ORBBEC_SDK_METHOD:-deb}"
PREFIX="${PREFIX:-/usr/local}"
CACHE_DIR="${ORBBEC_SDK_DIR:-/tmp/orbbec-sdk}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "missing required command: $1" >&2
    exit 1
  }
}

run_priv() {
  if [[ -w "${PREFIX}" ]] || [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  else
    need_cmd sudo
    sudo "$@"
  fi
}

detect_deb_arch() {
  local m
  m="$(uname -m)"
  case "${m}" in
    x86_64|amd64) echo "amd64" ;;
    aarch64|arm64) echo "arm64" ;;
    *)
      echo "unsupported arch for Orbbec .deb: ${m} (use ORBBEC_SDK_METHOD=source)" >&2
      exit 1
      ;;
  esac
}

sdk_already_installed() {
  if [[ "${ORBBEC_FORCE:-0}" == "1" ]]; then
    return 1
  fi
  if [[ -f "${PREFIX}/include/libobsensor/ObSensor.hpp" ]] \
    || [[ -f /usr/include/libobsensor/ObSensor.hpp ]] \
    || [[ -f "${PREFIX}/lib/libOrbbecSDK.so" ]] \
    || [[ -f "${PREFIX}/lib64/libOrbbecSDK.so" ]] \
    || [[ -f /usr/local/lib/libOrbbecSDK.so ]]; then
    return 0
  fi
  if command -v dpkg >/dev/null 2>&1 && dpkg -s orbbecsdk >/dev/null 2>&1; then
    return 0
  fi
  return 1
}

install_udev_from_tree() {
  local root="$1"
  local rules_script=""
  for cand in \
    "${root}/scripts/env_setup/install_udev_rules.sh" \
    "${root}/misc/scripts/install_udev_rules.sh"; do
    if [[ -f "${cand}" ]]; then
      rules_script="${cand}"
      break
    fi
  done
  if [[ -z "${rules_script}" ]]; then
    echo "udev install script not found under ${root}; skip" >&2
    return 0
  fi
  echo "== Installing Orbbec udev rules =="
  run_priv chmod +x "${rules_script}"
  run_priv bash "${rules_script}"
  if command -v udevadm >/dev/null 2>&1; then
    run_priv udevadm control --reload || true
    run_priv udevadm trigger || true
  fi
}

resolve_release_tag() {
  if [[ -n "${ORBBEC_SDK_VERSION:-}" ]]; then
    echo "${ORBBEC_SDK_VERSION}"
    return
  fi
  need_cmd curl
  need_cmd python3
  local json
  json="$(curl -fsSL -A 'autodriver-install-orbbec' "${REPO_API}/latest")"
  python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])' <<<"${json}"
}

download_deb() {
  need_cmd curl
  need_cmd python3
  local tag arch asset url out json pair
  tag="$(resolve_release_tag)"
  arch="$(detect_deb_arch)"
  mkdir -p "${CACHE_DIR}"
  # Logs must go to stderr: caller captures stdout as the .deb path.
  echo "== Resolving OrbbecSDK ${tag} (${arch}.deb) ==" >&2
  json="$(curl -fsSL -A 'autodriver-install-orbbec' "${REPO_API}/tags/${tag}")"
  pair="$(printf '%s' "${json}" | python3 -c '
import json, sys
arch = sys.argv[1]
rel = json.load(sys.stdin)
suffix = f"_{arch}.deb"
cands = []
for a in rel.get("assets", []):
    name = a.get("name") or ""
    if name.startswith("OrbbecSDK_") and name.endswith(suffix):
        cands.append((name, a["browser_download_url"]))
if not cands:
    sys.exit(f"no matching .deb asset for arch {arch}")
cands.sort(key=lambda x: (len(x[0]), x[0]))
print(cands[0][0] + "\t" + cands[0][1])
' "${arch}")"
  asset="${pair%%$'\t'*}"
  url="${pair#*$'\t'}"
  out="${CACHE_DIR}/${asset}"
  if [[ ! -f "${out}" ]]; then
    echo "== Downloading ${url} ==" >&2
    curl -fL --retry 3 -o "${out}.partial" "${url}"
    mv "${out}.partial" "${out}"
  else
    echo "== Using cached ${out} ==" >&2
  fi
  ORBBEC_DEB_PATH="${out}"
}

install_deb() {
  local deb="$1"
  need_cmd dpkg
  if [[ ! -f "${deb}" ]]; then
    echo "deb not found: ${deb}" >&2
    exit 1
  fi
  echo "== Installing ${deb} (may need sudo) =="
  run_priv dpkg -i "${deb}" || {
    echo "dpkg reported issues; attempting apt-get -f install" >&2
    if command -v apt-get >/dev/null 2>&1; then
      run_priv apt-get install -f -y
      run_priv dpkg -i "${deb}"
    else
      exit 1
    fi
  }
}

download_source() {
  need_cmd git
  local tag ref
  tag="$(resolve_release_tag)"
  ref="${tag}"
  if [[ -d "${CACHE_DIR}/OrbbecSDK_v2/.git" ]]; then
    echo "== Updating ${CACHE_DIR}/OrbbecSDK_v2 (${ref}) ==" >&2
    git -C "${CACHE_DIR}/OrbbecSDK_v2" fetch --depth 1 origin "refs/tags/${ref}:refs/tags/${ref}" \
      || git -C "${CACHE_DIR}/OrbbecSDK_v2" fetch --depth 1 origin "${ref}" || true
    git -C "${CACHE_DIR}/OrbbecSDK_v2" checkout -q "${ref}"
  else
    echo "== Cloning ${REPO_URL} (${ref}) ==" >&2
    rm -rf "${CACHE_DIR}/OrbbecSDK_v2"
    mkdir -p "${CACHE_DIR}"
    git clone --depth 1 --branch "${ref}" "${REPO_URL}" "${CACHE_DIR}/OrbbecSDK_v2" \
      || git clone --depth 1 "${REPO_URL}" "${CACHE_DIR}/OrbbecSDK_v2"
    git -C "${CACHE_DIR}/OrbbecSDK_v2" checkout -q "${ref}" || true
  fi
  ORBBEC_SRC_PATH="${CACHE_DIR}/OrbbecSDK_v2"
}

install_from_source() {
  local src="$1"
  need_cmd cmake
  need_cmd make
  need_cmd g++
  if [[ ! -f "${src}/CMakeLists.txt" ]]; then
    echo "OrbbecSDK source invalid: ${src}" >&2
    exit 1
  fi
  local build_dir="${src}/build-autodriver"
  echo "== Building OrbbecSDK in ${build_dir} =="
  cmake -S "${src}" -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTS=OFF
  cmake --build "${build_dir}" -j"$(nproc)"
  echo "== Installing to ${PREFIX} =="
  run_priv cmake --install "${build_dir}" --prefix "${PREFIX}"
  if [[ "${ORBBEC_SKIP_UDEV:-0}" != "1" ]]; then
    install_udev_from_tree "${src}"
  fi
}

print_done() {
  echo
  echo "Orbbec SDK installed."
  echo "Rebuild autodriver with:"
  echo "  cmake -S . -B build -DAUTODRIVER_WITH_ORBBEC=ON"
  echo "  cmake --build build -j\"\$(nproc)\" --target autodriver"
  echo "Confirm configure status: autodriver: OrbbecSDK enabled"
  if [[ "${ORBBEC_SKIP_UDEV:-0}" != "1" ]]; then
    echo "Replug the camera if permission denied on open (udev)."
  fi
}

# --- main ---
if sdk_already_installed; then
  echo "Orbbec SDK already present under ${PREFIX} (or dpkg orbbecsdk); skip."
  echo "Set ORBBEC_FORCE=1 to reinstall."
  print_done
  exit 0
fi

ARG="${1:-}"

if [[ -n "${ARG}" && -f "${ARG}" && "${ARG}" == *.deb ]]; then
  install_deb "${ARG}"
  print_done
  exit 0
fi

if [[ -n "${ARG}" && -d "${ARG}" && -f "${ARG}/CMakeLists.txt" ]]; then
  install_from_source "${ARG}"
  print_done
  exit 0
fi

case "${METHOD}" in
  deb)
    ORBBEC_DEB_PATH=""
    download_deb
    install_deb "${ORBBEC_DEB_PATH}"
    # Official .deb usually ships udev; still try if source tree was cached.
    if [[ "${ORBBEC_SKIP_UDEV:-0}" != "1" && -d "${CACHE_DIR}/OrbbecSDK_v2" ]]; then
      install_udev_from_tree "${CACHE_DIR}/OrbbecSDK_v2" || true
    fi
    ;;
  source)
    ORBBEC_SRC_PATH=""
    download_source
    install_from_source "${ORBBEC_SRC_PATH}"
    ;;
  *)
    echo "unknown ORBBEC_SDK_METHOD=${METHOD} (use deb|source)" >&2
    exit 1
    ;;
esac

print_done
