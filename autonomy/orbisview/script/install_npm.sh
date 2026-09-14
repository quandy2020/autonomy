#!/usr/bin/env bash
# Install Node.js + npm (≥18) for OrbisView frontend CMake / Vite builds.
#
# Usage:
#   ./script/install_npm.sh
#   NODE_MAJOR=22 ./script/install_npm.sh
#   PREFIX=/usr/local ./script/install_npm.sh   # tarball fallback prefix
#
# Env:
#   NODE_MAJOR   Major version (default 20 LTS)
#   PREFIX       Install prefix for official tarball path (default /usr/local)
#   FORCE=1      Reinstall even if an adequate node/npm is already present

set -euo pipefail

NODE_MAJOR="${NODE_MAJOR:-20}"
PREFIX="${PREFIX:-/usr/local}"
FORCE="${FORCE:-0}"
MIN_MAJOR=18

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "missing required command: $1" >&2
    exit 1
  }
}

run_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  elif command -v sudo >/dev/null 2>&1; then
    sudo "$@"
  else
    echo "need root (or sudo) to install system packages" >&2
    exit 1
  fi
}

node_major() {
  local v
  v="$(node -v 2>/dev/null || true)"
  v="${v#v}"
  echo "${v%%.*}"
}

already_ok() {
  [[ "${FORCE}" == "1" ]] && return 1
  command -v node >/dev/null 2>&1 || return 1
  command -v npm >/dev/null 2>&1 || return 1
  local major
  major="$(node_major)"
  [[ -n "${major}" && "${major}" -ge "${MIN_MAJOR}" ]]
}

install_nodesource_apt() {
  need_cmd curl
  need_cmd apt-get
  echo "== Installing Node.js ${NODE_MAJOR}.x via NodeSource =="
  export DEBIAN_FRONTEND=noninteractive
  run_root apt-get update -qq
  run_root apt-get install -y -qq ca-certificates curl gnupg
  curl -fsSL "https://deb.nodesource.com/setup_${NODE_MAJOR}.x" \
    | run_root bash -
  # Avoid hanging quiet installs on large apt indexes — show progress.
  run_root apt-get install -y nodejs
}

install_official_tarball() {
  need_cmd curl
  need_cmd tar
  need_cmd uname

  local arch os file url tmp ver
  os="$(uname -s | tr '[:upper:]' '[:lower:]')"
  arch="$(uname -m)"
  case "${arch}" in
    x86_64|amd64) arch="x64" ;;
    aarch64|arm64) arch="arm64" ;;
    *)
      echo "unsupported arch: ${arch}" >&2
      exit 1
      ;;
  esac
  [[ "${os}" == "linux" ]] || {
    echo "tarball installer supports linux only (got ${os}); install Node ≥${MIN_MAJOR} manually" >&2
    exit 1
  }

  # Resolve latest ${NODE_MAJOR}.x from the official dist channel
  file="$(curl -fsSL "https://nodejs.org/dist/latest-v${NODE_MAJOR}.x/SHASUMS256.txt" \
    | awk -v a="${arch}" '$2 ~ ("^node-v.+-linux-" a "\\.tar\\.xz$") { print $2; exit }')"
  [[ -n "${file}" ]] || {
    echo "could not resolve Node ${NODE_MAJOR}.x linux-${arch} tarball from nodejs.org" >&2
    exit 1
  }
  ver="${file#node-}"
  ver="${ver%-linux-*}"
  url="https://nodejs.org/dist/latest-v${NODE_MAJOR}.x/${file}"
  tmp="$(mktemp -d)"
  echo "== Downloading ${url} (${ver}) =="
  curl -fsSL "${url}" -o "${tmp}/${file}"
  echo "== Extracting to ${PREFIX} =="
  run_root tar -xJf "${tmp}/${file}" -C "${PREFIX}" --strip-components=1
  rm -rf "${tmp}"
}

main() {
  if already_ok; then
    echo "Node $(node -v) / npm $(npm -v) already OK (≥${MIN_MAJOR}); skip"
    echo "Tip: FORCE=1 $0 to reinstall"
    exit 0
  fi

  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    case "${ID:-}:${ID_LIKE:-}" in
      debian:*|ubuntu:*|*debian*|*ubuntu*)
        install_nodesource_apt
        ;;
      *)
        install_official_tarball
        ;;
    esac
  else
    install_official_tarball
  fi

  hash -r 2>/dev/null || true
  command -v node >/dev/null 2>&1 || {
    echo "node not on PATH after install; ensure ${PREFIX}/bin is in PATH" >&2
    exit 1
  }
  command -v npm >/dev/null 2>&1 || {
    echo "npm not on PATH after install" >&2
    exit 1
  }

  local major
  major="$(node_major)"
  if [[ -z "${major}" || "${major}" -lt "${MIN_MAJOR}" ]]; then
    echo "installed node $(node -v) is older than ${MIN_MAJOR}" >&2
    exit 1
  fi

  echo "Installed Node $(node -v) / npm $(npm -v)"
  echo "Reconfigure CMake then: cmake --build <build> --target orbisview_frontend"
}

main "$@"
