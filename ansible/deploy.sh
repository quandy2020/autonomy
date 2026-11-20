#!/usr/bin/env bash
# Autonomy Ansible deployment entrypoint (single playbook: playbooks/site.yml).
#
# Modes:
#   full    编译/制品 + 配置 + 服务（默认）
#   build   仅编译安装
#   deploy  制品分发（等同 full + robots inventory）
#   push    仅同步配置并重启
#   check   预检
#   restart 重启服务
#
# Examples:
#   ./deploy.sh build
#   ./deploy.sh deploy robots -e autonomy_artifact_path=$PWD/../dist/autonomy.tar.gz
#   ./deploy.sh push robots -l robot02
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT}"

MODE="full"
PLAYBOOK="playbooks/site.yml"
INVENTORY="inventory/hosts.yml"
EXTRA_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./deploy.sh [MODE] [INVENTORY] [ansible-playbook options...]

Modes (all use playbooks/site.yml):
  full     Build or artifact + config + services (default)
  build    Compile and install only
  deploy   Artifact deploy to fleet (full + robots inventory)
  push     Sync config and restart services
  check    Preflight checks
  restart  Restart systemd units

Inventories:
  (omit)     inventory/hosts.yml (localhost)
  robots     inventory/robots/hosts.yml
  staging    inventory/staging/hosts.yml
  production inventory/production/hosts.yml

Examples:
  ./deploy.sh build
  ./deploy.sh deploy robots -e autonomy_artifact_path=$PWD/../dist/autonomy.tar.gz
  ./deploy.sh push robots -l robot01
  ./deploy.sh check robots
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if ! command -v ansible-playbook >/dev/null 2>&1; then
  echo "ansible-playbook not found. Install: pip install -r requirements-ci.txt" >&2
  exit 1
fi

case "${1:-}" in
  full|build|deploy|push|check|restart)
    MODE="$1"
    shift
  ;;
esac

case "${MODE}" in
  full) EXTRA_ARGS+=(-e "autonomy_play_mode=full") ;;
  build) EXTRA_ARGS+=(-e "autonomy_play_mode=build" -e "autonomy_deploy_mode=build") ;;
  deploy) EXTRA_ARGS+=(-e "autonomy_play_mode=full" -e "autonomy_deploy_mode=artifact") ;;
  push) EXTRA_ARGS+=(-e "autonomy_play_mode=push") ;;
  check) EXTRA_ARGS+=(-e "autonomy_play_mode=check") ;;
  restart) EXTRA_ARGS+=(-e "autonomy_play_mode=restart") ;;
  *)
    echo "Unknown mode: ${MODE}" >&2
    usage
    exit 1
  ;;
esac

if [[ $# -gt 0 && "${1}" != -* ]]; then
  if [[ ! -f "inventory/${1}/hosts.yml" ]]; then
    echo "Unknown inventory: ${1}" >&2
    exit 1
  fi
  INVENTORY="inventory/${1}/hosts.yml"
  shift
elif [[ -n "${AUTONOMY_DEPLOY_ENV:-}" ]]; then
  INVENTORY="inventory/${AUTONOMY_DEPLOY_ENV}/hosts.yml"
  if [[ ! -f "${INVENTORY}" ]]; then
    echo "Unknown AUTONOMY_DEPLOY_ENV=${AUTONOMY_DEPLOY_ENV}" >&2
    exit 1
  fi
fi

if [[ "${MODE}" == "deploy" && "${INVENTORY}" == "inventory/hosts.yml" ]]; then
  INVENTORY="inventory/robots/hosts.yml"
fi

if [[ -f "${ROOT}/.vault_pass" ]]; then
  EXTRA_ARGS+=(--vault-password-file "${ROOT}/.vault_pass")
fi

if [[ "${INVENTORY}" == "inventory/hosts.yml" && "${MODE}" != "check" && -t 0 ]]; then
  ask_become=false
  for arg in "$@"; do
    if [[ "${arg}" == "-K" || "${arg}" == "--ask-become-pass" || "${arg}" == "--syntax-check" || "${arg}" == --list-* ]]; then
      ask_become=true
      break
    fi
  done
  if [[ "${ask_become}" == "false" ]]; then
    EXTRA_ARGS+=(--ask-become-pass)
  fi
fi

if [[ "${INVENTORY}" == "inventory/production/hosts.yml" ]]; then
  PLAYBOOK="playbooks/production.yml"
elif [[ "${INVENTORY}" == "inventory/staging/hosts.yml" ]]; then
  PLAYBOOK="playbooks/staging.yml"
elif [[ "${INVENTORY}" == "inventory/sim/hosts.yml" ]]; then
  PLAYBOOK="playbooks/sim.yml"
fi

"${ROOT}/scripts/install_collection.sh"

echo "==> mode=${MODE} inventory=${INVENTORY} playbook=${PLAYBOOK}"
exec ansible-playbook -i "${INVENTORY}" "${PLAYBOOK}" "${EXTRA_ARGS[@]}" "$@"
