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

if ! command -v ansible-playbook >/dev/null 2>&1; then
  echo "ansible-playbook not found. Install: pip install ansible ansible-lint" >&2
  exit 1
fi

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

case "${1:-}" in
  full|build|deploy|push|check|restart)
    MODE="$1"
    shift
  ;;
esac

case "${MODE}" in
  full) ;;
  build) EXTRA_ARGS+=(-e "autonomy_play_mode=build") ;;
  deploy) ;;
  push) EXTRA_ARGS+=(-e "autonomy_play_mode=push") ;;
  check) EXTRA_ARGS+=(-e "autonomy_play_mode=check") ;;
  restart) EXTRA_ARGS+=(-e "autonomy_play_mode=restart") ;;
  *)
    echo "Unknown mode: ${MODE}" >&2
    usage
    exit 1
  ;;
esac

if [[ -n "${AUTONOMY_DEPLOY_ENV:-}" ]]; then
  if [[ -f "inventory/${AUTONOMY_DEPLOY_ENV}/hosts.yml" ]]; then
    INVENTORY="inventory/${AUTONOMY_DEPLOY_ENV}/hosts.yml"
  else
    echo "Unknown AUTONOMY_DEPLOY_ENV=${AUTONOMY_DEPLOY_ENV}" >&2
    exit 1
  fi
elif [[ $# -gt 0 && "${1}" != -* && -f "inventory/${1}/hosts.yml" ]]; then
  INVENTORY="inventory/${1}/hosts.yml"
  shift
fi

if [[ "${MODE}" == "deploy" && "${INVENTORY}" == "inventory/hosts.yml" ]]; then
  INVENTORY="inventory/robots/hosts.yml"
fi

if [[ -f "${ROOT}/.vault_pass" ]]; then
  EXTRA_ARGS+=(--vault-password-file "${ROOT}/.vault_pass")
fi

if [[ "${INVENTORY}" == "inventory/hosts.yml" ]]; then
  ask_become=false
  for arg in "$@"; do
    if [[ "${arg}" == "-K" || "${arg}" == "--ask-become-pass" ]]; then
      ask_become=true
      break
    fi
  done
  if [[ "${ask_become}" == "false" ]]; then
    EXTRA_ARGS+=(--ask-become-pass)
  fi
fi

echo "==> mode=${MODE} inventory=${INVENTORY} playbook=${PLAYBOOK}"
exec ansible-playbook -i "${INVENTORY}" "${PLAYBOOK}" "${EXTRA_ARGS[@]}" "$@"
