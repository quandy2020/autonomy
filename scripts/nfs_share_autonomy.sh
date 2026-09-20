#!/usr/bin/env bash
# One-shot NFS share: x86 host exports autonomy → Firefly mounts it.
# Usage:
#   Host (x86):   sudo bash scripts/nfs_share_autonomy.sh server
#   Firefly:      bash scripts/nfs_share_autonomy.sh client
#   Or both sides: see "all" help below.
set -euo pipefail

EXPORT_PATH="${EXPORT_PATH:-/home/quandy/workspace/github/autonomy/src/autonomy}"
CLIENT_NET="${CLIENT_NET:-192.168.234.0/24}"
HOST_IP="${HOST_IP:-192.168.234.50}"
MOUNT_POINT="${MOUNT_POINT:-/home/firefly/autonomy}"
WS_LOCAL="${WS_LOCAL:-/home/firefly/autonomy_ws}"

die() { echo "ERROR: $*" >&2; exit 1; }
info() { echo "==> $*"; }

setup_server() {
  [[ "$(id -u)" -eq 0 ]] || die "server mode needs root (sudo)"
  [[ -d "${EXPORT_PATH}" ]] || die "export path missing: ${EXPORT_PATH}"

  if ! dpkg -l nfs-kernel-server 2>/dev/null | grep -q '^ii'; then
    info "install nfs-kernel-server"
    apt-get update -qq
    apt-get install -y nfs-kernel-server
  fi

  # Map client UIDs to host quandy (1000) so Firefly can write without root_squash pain.
  local line="${EXPORT_PATH} ${CLIENT_NET}(rw,sync,no_subtree_check,all_squash,anonuid=1000,anongid=1000)"
  cp -a /etc/exports "/etc/exports.bak.$(date +%Y%m%d%H%M%S)"
  if grep -qF "${EXPORT_PATH}" /etc/exports; then
    grep -vF "${EXPORT_PATH}" /etc/exports > /tmp/exports.new
    echo "${line}" >> /tmp/exports.new
    cp /tmp/exports.new /etc/exports
  else
    echo "${line}" >> /etc/exports
  fi

  # NFS needs traverse permission on path components.
  local p="/"
  IFS=/ read -ra parts <<< "${EXPORT_PATH}"
  for part in "${parts[@]}"; do
    [[ -z "${part}" ]] && continue
    p="${p%/}/${part}"
    chmod a+rx "${p}" 2>/dev/null || true
  done

  exportfs -ra
  systemctl enable --now nfs-server
  info "exports:"
  exportfs -v
  info "showmount:"
  showmount -e "${HOST_IP}" || showmount -e 127.0.0.1
  cat <<EOF

NFS server ready.
  Export : ${EXPORT_PATH}
  Allow  : ${CLIENT_NET}
  Host IP: ${HOST_IP}

On Firefly run:
  HOST_IP=${HOST_IP} bash /path/to/nfs_share_autonomy.sh client
  # or after mount, copy this script via scp and run client.
EOF
}

setup_client() {
  command -v mount.nfs >/dev/null 2>&1 || {
    info "install nfs-common"
    sudo apt-get update -qq
    sudo apt-get install -y nfs-common
  }

  mkdir -p "${MOUNT_POINT}"
  if mountpoint -q "${MOUNT_POINT}"; then
    info "already mounted: ${MOUNT_POINT}"
  else
    info "mount ${HOST_IP}:${EXPORT_PATH} → ${MOUNT_POINT}"
    sudo mount -t nfs -o vers=3,nolock,tcp,soft,timeo=50 \
      "${HOST_IP}:${EXPORT_PATH}" "${MOUNT_POINT}" \
      || sudo mount -t nfs -o vers=4,nolock,tcp,soft,timeo=50 \
      "${HOST_IP}:${EXPORT_PATH}" "${MOUNT_POINT}"
  fi

  df -h "${MOUNT_POINT}"
  ls "${MOUNT_POINT}" | head

  # Local build workspace (build/install stay off NFS)
  mkdir -p "${WS_LOCAL}/src" "${WS_LOCAL}/build" "${WS_LOCAL}/install" "${WS_LOCAL}/log"
  ln -sfn "${MOUNT_POINT}" "${WS_LOCAL}/src/autonomy"

  # Optional fstab (idempotent)
  local fstab_line="${HOST_IP}:${EXPORT_PATH} ${MOUNT_POINT} nfs vers=3,nolock,tcp,soft,_netdev,nofail 0 0"
  if ! grep -qF "${MOUNT_POINT}" /etc/fstab 2>/dev/null; then
    info "append fstab (needs sudo)"
    echo "${fstab_line}" | sudo tee -a /etc/fstab >/dev/null
  fi

  cat <<EOF

NFS client ready.
  Mount : ${MOUNT_POINT}
  Build : ${WS_LOCAL}  (src/autonomy → NFS)

Build on Firefly:
  cd ${WS_LOCAL}
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
EOF
}

usage() {
  cat <<EOF
Usage: $0 server|client

  server  — run on x86 host with sudo (exports autonomy tree)
  client  — run on Firefly (mounts + local autonomy_ws)

Env overrides:
  EXPORT_PATH  HOST_IP  CLIENT_NET  MOUNT_POINT  WS_LOCAL
EOF
}

main() {
  local cmd="${1:-}"
  case "${cmd}" in
    server) setup_server ;;
    client) setup_client ;;
    *) usage; exit 1 ;;
  esac
}

main "$@"
