#!/usr/bin/env bash
# One-click NFS: x86 host exports autonomy → Firefly mounts it.
#
# From the x86 host (recommended):
#   bash scripts/share_nfs_workspace.sh          # same as: all
#   bash scripts/share_nfs_workspace.sh all
#   bash scripts/share_nfs_workspace.sh status
#   bash scripts/share_nfs_workspace.sh down
#
# Manual split:
#   sudo bash scripts/share_nfs_workspace.sh server
#   bash scripts/share_nfs_workspace.sh client    # on Firefly
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

EXPORT_PATH="${EXPORT_PATH:-${REPO_ROOT}}"
CLIENT_NET="${CLIENT_NET:-192.168.234.0/24}"
# Default lab LAN; overridden by env or auto-detect in one_click/setup_server.
HOST_IP="${HOST_IP:-}"
FIREFLY_HOST="${FIREFLY_HOST:-firefly@192.168.234.1}"
FIREFLY_SSH_KEY="${FIREFLY_SSH_KEY:-${HOME}/.ssh/id_rsa}"
MOUNT_POINT="${MOUNT_POINT:-/home/firefly/autonomy}"
WS_LOCAL="${WS_LOCAL:-/home/firefly/autonomy_ws}"
SSH_OPTS=(
  -o BatchMode=yes
  -o StrictHostKeyChecking=accept-new
  -o ConnectTimeout=10
  -o ServerAliveInterval=5
  -o ServerAliveCountMax=3
)
[[ -f "${FIREFLY_SSH_KEY}" ]] && SSH_OPTS+=(-i "${FIREFLY_SSH_KEY}")

die() { echo "ERROR: $*" >&2; exit 1; }
info() { echo "==> $*"; }
ok() { echo "OK  $*"; }

# Pick host IP on CLIENT_NET (e.g. 192.168.234.x). Prefer env HOST_IP.
resolve_host_ip() {
  if [[ -n "${HOST_IP}" ]]; then
    return 0
  fi
  local ip=""
  local probe="${BOARD_IP:-}"
  if [[ -z "${probe}" && "${FIREFLY_HOST}" == *@* ]]; then
    probe="${FIREFLY_HOST##*@}"
  fi
  probe="${probe:-192.168.234.1}"
  # Prefer src address used to reach the board.
  ip="$(ip -4 route get "${probe}" 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}' || true)"
  if [[ -z "${ip}" ]]; then
    ip="$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -E '^192\.168\.234\.' | head -1 || true)"
  fi
  if [[ -z "${ip}" ]]; then
    ip="$(ip -4 -o addr show scope global 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | grep -E '^192\.168\.234\.' | head -1 || true)"
  fi
  HOST_IP="${ip:-192.168.234.50}"
  info "HOST_IP=${HOST_IP} (set --host-ip / HOST_IP= to override)"
}

need_sudo() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  elif sudo -n true 2>/dev/null; then
    sudo "$@"
  else
    # Interactive password prompt (one-click from a real terminal).
    sudo "$@"
  fi
}

ssh_ff() {
  ssh "${SSH_OPTS[@]}" "${FIREFLY_HOST}" "$@"
}

setup_server() {
  resolve_host_ip
  [[ -d "${EXPORT_PATH}" ]] || die "export path missing: ${EXPORT_PATH}"

  if ! dpkg -l nfs-kernel-server 2>/dev/null | grep -q '^ii'; then
    info "install nfs-kernel-server"
    need_sudo apt-get update -qq
    need_sudo apt-get install -y nfs-kernel-server
  fi

  # Map client UIDs to host owner so Firefly can write without root_squash pain.
  local owner_uid owner_gid
  owner_uid="$(stat -c %u "${EXPORT_PATH}")"
  owner_gid="$(stat -c %g "${EXPORT_PATH}")"
  local line="${EXPORT_PATH} ${CLIENT_NET}(rw,sync,no_subtree_check,all_squash,anonuid=${owner_uid},anongid=${owner_gid})"

  need_sudo cp /etc/exports "/tmp/exports.bak.$(date +%Y%m%d%H%M%S)" || true
  if need_sudo grep -qF "${EXPORT_PATH}" /etc/exports 2>/dev/null; then
    need_sudo bash -c "grep -vF $(printf '%q' "${EXPORT_PATH}") /etc/exports > /tmp/exports.new && printf '%s\n' $(printf '%q' "${line}") >> /tmp/exports.new && cp /tmp/exports.new /etc/exports"
  else
    printf '%s\n' "${line}" | need_sudo tee -a /etc/exports >/dev/null
  fi

  # NFS needs traverse permission on path components.
  local p="/"
  local IFS=/
  # shellcheck disable=SC2206
  local parts=(${EXPORT_PATH})
  for part in "${parts[@]}"; do
    [[ -z "${part}" ]] && continue
    p="${p%/}/${part}"
    need_sudo chmod a+rx "${p}" 2>/dev/null || true
  done

  need_sudo exportfs -ra
  need_sudo systemctl enable --now nfs-server
  # Some distros use nfs-kernel-server unit name.
  need_sudo systemctl enable --now nfs-kernel-server 2>/dev/null || true

  info "exports:"
  need_sudo exportfs -v
  info "showmount:"
  showmount -e "${HOST_IP}" 2>/dev/null || showmount -e 127.0.0.1 || true
  ok "NFS server ready  ${EXPORT_PATH} → ${CLIENT_NET}  (host ${HOST_IP})"
}

setup_client_local() {
  # Runs on Firefly (or any client).
  command -v mount.nfs >/dev/null 2>&1 || {
    info "install nfs-common"
    need_sudo apt-get update -qq
    need_sudo apt-get install -y nfs-common
  }

  mkdir -p "${MOUNT_POINT}"
  if mountpoint -q "${MOUNT_POINT}"; then
    info "already mounted: ${MOUNT_POINT}"
  else
    info "mount ${HOST_IP}:${EXPORT_PATH} → ${MOUNT_POINT}"
    need_sudo mount -t nfs -o vers=3,nolock,tcp,soft,timeo=50 \
      "${HOST_IP}:${EXPORT_PATH}" "${MOUNT_POINT}" \
      || need_sudo mount -t nfs -o vers=4,nolock,tcp,soft,timeo=50 \
      "${HOST_IP}:${EXPORT_PATH}" "${MOUNT_POINT}"
  fi

  df -h "${MOUNT_POINT}"
  ls "${MOUNT_POINT}" | head

  mkdir -p "${WS_LOCAL}/src" "${WS_LOCAL}/build" "${WS_LOCAL}/install" "${WS_LOCAL}/log"
  ln -sfn "${MOUNT_POINT}" "${WS_LOCAL}/src/autonomy"

  local fstab_line="${HOST_IP}:${EXPORT_PATH} ${MOUNT_POINT} nfs vers=3,nolock,tcp,soft,_netdev,nofail 0 0"
  if ! grep -qF "${MOUNT_POINT}" /etc/fstab 2>/dev/null; then
    info "append fstab"
    echo "${fstab_line}" | need_sudo tee -a /etc/fstab >/dev/null
  fi

  ok "NFS client ready  ${MOUNT_POINT}  (build @ ${WS_LOCAL})"
}

setup_client_remote() {
  info "SSH ${FIREFLY_HOST} → mount client"
  # Prefer running the script from the export once mounted; bootstrap via stdin first.
  # shellcheck disable=SC2029
  ssh_ff "bash -s" <<EOF
set -euo pipefail
export HOST_IP='${HOST_IP}'
export EXPORT_PATH='${EXPORT_PATH}'
export MOUNT_POINT='${MOUNT_POINT}'
export WS_LOCAL='${WS_LOCAL}'
$(declare -f die info ok need_sudo setup_client_local)
setup_client_local
EOF
}

teardown_client_remote() {
  info "umount on ${FIREFLY_HOST}"
  ssh_ff "bash -s" <<EOF
set -euo pipefail
MP='${MOUNT_POINT}'
if mountpoint -q "\${MP}" 2>/dev/null; then
  sudo umount "\${MP}" || sudo umount -l "\${MP}" || true
fi
echo "umount done: \${MP}"
EOF
}

teardown_server() {
  resolve_host_ip
  info "unexport ${EXPORT_PATH}"
  need_sudo exportfs -u "${EXPORT_PATH}" 2>/dev/null || true
  need_sudo exportfs -ra
  ok "server unexport done (nfs-server still running)"
}

show_status() {
  resolve_host_ip
  echo "--- host ---"
  echo "HOST_IP=${HOST_IP}  EXPORT_PATH=${EXPORT_PATH}"
  need_sudo exportfs -v 2>/dev/null || echo "(no exports / need sudo)"
  showmount -e "${HOST_IP}" 2>/dev/null || true
  echo "--- firefly (${FIREFLY_HOST}) ---"
  if ssh_ff "bash -s" <<EOF
set +e
echo "mount:"
mount | grep -E 'nfs|${MOUNT_POINT}' || echo '(not mounted)'
df -h '${MOUNT_POINT}' 2>/dev/null || true
ls -la '${WS_LOCAL}/src/autonomy' 2>/dev/null || true
ls '${MOUNT_POINT}' 2>/dev/null | head -5 || true
EOF
  then
    :
  else
    echo "(SSH failed — is Firefly up?)"
  fi
}

one_click() {
  resolve_host_ip
  info "one-click NFS: server@${HOST_IP} + client@${FIREFLY_HOST}"
  setup_server
  # Brief wait for rpcbind/nfsd
  sleep 1
  setup_client_remote
  echo
  show_status
  cat <<EOF

Build on Firefly:
  cd ${WS_LOCAL}
  cmake -S src/autonomy -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/usr/local \\
    -DBUILD_AUTOVIZ=OFF -DBUILD_ORBISVIEW=OFF -DBUILD_DOCS=OFF
EOF
}

usage() {
  cat <<EOF
Usage: $0 [options] [all|server|client|status|down]

  all      — one-click from x86 host (default): export + SSH mount board
  server   — NFS export only (this machine)
  client   — NFS mount only (run on the board)
  status   — show export + remote mount
  down     — umount board + unexport path

Options (override env; different boards = different --board-ip):
  --board-ip IP          Board address (sets FIREFLY_HOST user@IP)
  --board-host USER@IP   Full SSH target (e.g. firefly@192.168.234.20)
  --board-user USER      SSH user (default: firefly)
  --host-ip IP           This PC's IP as seen by the board (NFS server)
  --client-net CIDR      Allowed NFS clients (default: 192.168.234.0/24)
  --export PATH          Path to export (default: repo root)
  --mount PATH           Board mount point
  --ws PATH              Board local workspace (build lives here)
  --ssh-key PATH         SSH private key
  --board-profile NAME   Load scripts/nfs_boards/NAME.env
  -h, --help             This help

Env (same meaning): FIREFLY_HOST HOST_IP CLIENT_NET EXPORT_PATH
  MOUNT_POINT WS_LOCAL FIREFLY_SSH_KEY BOARD_USER

Examples:
  $0 all --board-ip 192.168.234.1
  $0 all --board-ip 192.168.234.20 --board-user firefly
  $0 status --board-host root@10.0.0.5 --host-ip 10.0.0.1 --client-net 10.0.0.0/24
  $0 all --board-profile lab-a

See scripts/README.md for full NFS + board build steps.
EOF
}

load_board_profile() {
  local name="$1"
  local file="${SCRIPT_DIR}/nfs_boards/${name}.env"
  [[ -f "${file}" ]] || die "board profile not found: ${file}"
  info "load board profile: ${file}"
  # shellcheck disable=SC1090
  set -a
  # shellcheck disable=SC1090
  source "${file}"
  set +a
}

apply_board_user_ip() {
  # If BOARD_IP / BOARD_USER set, synthesize FIREFLY_HOST.
  local user="${BOARD_USER:-firefly}"
  if [[ -n "${BOARD_IP:-}" ]]; then
    FIREFLY_HOST="${user}@${BOARD_IP}"
  fi
}

parse_args() {
  CMD="all"
  local -a rest=()
  # Pass 1: load profile(s) first so later flags override.
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --board-profile)
        load_board_profile "$2"
        shift 2
        ;;
      *)
        rest+=("$1")
        shift
        ;;
    esac
  done
  # Pass 2: command + overrides
  set -- "${rest[@]}"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      all|up|one|one-click|server|client|status|down|stop|help)
        CMD="$1"
        shift
        ;;
      -h|--help)
        CMD="help"
        shift
        ;;
      --board-ip)
        BOARD_IP="$2"
        shift 2
        ;;
      --board-host)
        FIREFLY_HOST="$2"
        shift 2
        ;;
      --board-user)
        BOARD_USER="$2"
        shift 2
        ;;
      --host-ip)
        HOST_IP="$2"
        shift 2
        ;;
      --client-net)
        CLIENT_NET="$2"
        shift 2
        ;;
      --export)
        EXPORT_PATH="$2"
        shift 2
        ;;
      --mount)
        MOUNT_POINT="$2"
        shift 2
        ;;
      --ws)
        WS_LOCAL="$2"
        shift 2
        ;;
      --ssh-key)
        FIREFLY_SSH_KEY="$2"
        SSH_OPTS=(-o BatchMode=yes -o StrictHostKeyChecking=accept-new
          -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3)
        [[ -f "${FIREFLY_SSH_KEY}" ]] && SSH_OPTS+=(-i "${FIREFLY_SSH_KEY}")
        shift 2
        ;;
      --board-profile)
        # Already handled in pass 1
        shift 2
        ;;
      *)
        die "unknown argument: $1 (try --help)"
        ;;
    esac
  done
  apply_board_user_ip
}

main() {
  parse_args "$@"
  case "${CMD}" in
    all|up|one|one-click) one_click ;;
    server) setup_server ;;
    client) setup_client_local ;;
    status) show_status ;;
    down|stop) teardown_client_remote; teardown_server ;;
    help) usage ;;
    *) usage; exit 1 ;;
  esac
}

main "$@"
