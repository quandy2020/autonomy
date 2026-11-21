#!/usr/bin/env bash
# Autonomy's operator entrypoint. See docs/cli.md for supported commands.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly ROOT

# Highlight sections and commands without loading controller tooling.
usage() {
  local line
  while IFS= read -r line; do
    case "${line}" in
      Usage:*|Modes:|Inventories:|'Build and release options:'|'Common Ansible options:'|Output:|Examples:|'  # '*)
        log_message info "${line}"
        ;;
      '  ./deploy.sh '*|'    -e '*|'  archive='*|'  sha='*) log_message example "${line}" ;;
      *) printf '%s\n' "${line}" ;;
    esac
  done <<'EOF'
Usage: ./deploy.sh [MODE] [INVENTORY] [options...]
       ./deploy.sh cross-build [options...]
       ./deploy.sh style [PATH ...] [output options...]
       ./deploy.sh --help

Modes:
  full           Build from source or deploy an artifact as configured by the inventory.
                 Apply application configuration and manage services as configured (default).
  build          Compile on the selected hosts and install a full build. --module is compile-only.
  deploy         Activate an existing artifact and run its configuration and service rollout.
  release        Build and package in an existing Docker container, then deploy (vars/release.yml).
  cross-build    Cross-compile and package on the controller (vars/release-cross.yml).
                 With --module and no --base-artifact, only compile.
  cross-release  Preflight selected robots or staging hosts, cross-compile once and deploy.
                 Requires --limit for deployment and a matching cross Profile on every target.
  push           Sync application configuration. Restart systemd services on change if enabled.
  check          Check target OS, CPU architecture and free space on the root filesystem.
  plan           Run read-only deployment preflight on the targets. With -R, check the build's target ABI.
  verify         Check the active installation and run configured health checks without restarting services.
  restart        Restart the selected systemd services on system installs.
  style          Check first-party C/C++ against the repository's Google-based clang-format rules.
                 On failure, print initial diagnostics and the full report path. PATH is repository-relative.

Inventories:
  robots      inventory/robots/hosts.yml
  staging     inventory/staging/hosts.yml
  production inventory/production/hosts.yml
  sim         inventory/sim/hosts.yml
  demo        inventory/demo/hosts.yml (local user installation)

  With no inventory argument, use AUTONOMY_DEPLOY_ENV if set. Otherwise deploy,
  release and cross-release default to robots. Other Ansible modes use inventory/hosts.yml
  (localhost). Production release changes require the approved CI workflow.

Build and release options:
  -R, --release-config FILE  Select the build configuration for plan, release, cross-build or cross-release
  -I, --incremental          Reuse the cross-build cache in cross-build and cross-release.
                             Try delta transfer in deploy, release and cross-release.
                             Native build already reuses its build directory.
  -m, --module NAME          Repeat to select projects or autonomy/<domain> in build, cross-build,
                             cross-release or plan. Required build dependencies are included.
  -B, --base-artifact FILE   Use the active release archive as the module baseline.
                             Required with --module for cross-release and plan. Optional for cross-build.
  -S, --test                 In build mode, run CTest after compiling and before any full-build install

Common Ansible options:
  -l, --limit PATTERN       Limit the target hosts or groups
  -K, --ask-become-pass     Prompt for the privilege escalation password
  -C, --check               Preview supported Ansible tasks. For cross-release, use deploy -C with an artifact.
  -D, --diff                Show file differences. Pair with -C for a preview.
  -e, --extra-vars VARS     Pass additional Ansible variables
  --list-hosts              Show the selected hosts
  --syntax-check            Check playbook syntax

Output:
  -L, --color auto|always|never  Control terminal colors (default: auto)
  -o, --report FILE             Write the complete style report in style mode

Examples:
  # Preview targets and run preflight
  ./deploy.sh deploy robots -l robot01 --list-hosts
  ./deploy.sh deploy staging --syntax-check
  ./deploy.sh check robots -l robot01
  ./deploy.sh plan robots -l robot01 -R vars/release-cross.yml

  # Check, build and test during development
  ./deploy.sh style autonomy/control -o /tmp/autonomy-style.log
  ./deploy.sh build -K -S -e autonomy_install_prefix=/opt/autonomy-dev
  # Compile and test one module and its dependencies on staging
  ./deploy.sh build staging -l robot01 -K -m autonomy/control -S

  # Build and release native or ARM artifacts
  ./deploy.sh release robots -l robot01 -K
  ./deploy.sh release demo -R vars/release-demo.yml
  ./deploy.sh cross-build -I -R vars/release-cross.yml
  ./deploy.sh cross-build -I -m automsgs -m autonomy/control
  ./deploy.sh cross-release staging -l robot01 -K -I

  # Use the active release archive as the module baseline
  ./deploy.sh plan robots -l robot01 -K -m autonomy/control -B /absolute/path/to/current.tar.gz
  ./deploy.sh cross-release robots -l robot01 -K -m autonomy/control -B /absolute/path/to/current.tar.gz

  # Deploy an existing artifact and operate services
  archive=/absolute/path/to/autonomy.tar.gz
  sha=$(awk 'NR == 1 {print $1}' "$archive.sha256")
  ./deploy.sh deploy robots -l robot01 -K -I \
    -e "autonomy_artifact_path=$archive" \
    -e "autonomy_artifact_sha256=$sha"
  ./deploy.sh push robots -l robot01 -K -C -D
  ./deploy.sh push robots -l robot01 -K
  ./deploy.sh restart robots -l robot01 -K
  ./deploy.sh verify robots -l robot01 -e "autonomy_artifact_sha256=$sha"
  ./deploy.sh verify robots -l robot01 -K -e autonomy_show_release_retention=true

  # Force colored help when the output is redirected
  ./deploy.sh --help --color=always

See docs/cli.md for more combinations and prerequisites.
EOF
}

# Consume color options before mode selection so local-only modes and early errors share them.
configure_log_color() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -L|--color)
        log_color="${2:-}"
        if [[ $# -gt 1 ]]; then shift; fi
        ;;
      --color=*) log_color="${1#--color=}" ;;
      --) command_args+=("$@"); break ;;
      *) command_args+=("$1") ;;
    esac
    shift
  done

  case "${log_color}" in
    always) export ANSIBLE_FORCE_COLOR=1 ANSIBLE_NOCOLOR=0 ;;
    never) export ANSIBLE_FORCE_COLOR=0 ANSIBLE_NOCOLOR=1 ;;
    auto)
      export ANSIBLE_FORCE_COLOR=0
      if [[ -n "${NO_COLOR:-}" || "${TERM:-}" == dumb ]]; then
        export ANSIBLE_NOCOLOR=1
      fi
      ;;
    *) log_message error "--color requires auto, always or never."; exit 1 ;;
  esac
  export AUTONOMY_LOG_COLOR="${log_color}"
}

# Write stages and examples to stdout and errors to stderr. Check each stream's TTY.
log_message() {
  local fd=1 code=36
  case "$1" in
    error) fd=2; code=31 ;;
    example) code=32 ;;
  esac
  if [[ "${log_color}" == always ||
        ( "${log_color}" == auto && -t "${fd}" && "${TERM:-}" != dumb && -z "${NO_COLOR:-}" ) ]]; then
    printf '\033[%sm%s\033[0m\n' "${code}" "$2" >&"${fd}"
  else
    printf '%s\n' "$2" >&"${fd}"
  fi
}

# Set RELEASE_CONFIG and append mode defaults to EXTRA_ARGS.
configure_mode() {
  case "${MODE}" in
    full) EXTRA_ARGS+=(-e "autonomy_play_mode=full") ;;
    build) EXTRA_ARGS+=(-e "autonomy_play_mode=build" -e "autonomy_deploy_mode=build") ;;
    deploy) EXTRA_ARGS+=(-e "autonomy_play_mode=full" -e "autonomy_deploy_mode=artifact") ;;
    release|cross-release) EXTRA_ARGS+=(-e "autonomy_play_mode=full" -e "autonomy_deploy_mode=artifact" -e "autonomy_install_dependencies=false") ;;
    cross-build) ;;
    push) EXTRA_ARGS+=(-e "autonomy_play_mode=push") ;;
    check|plan|verify) EXTRA_ARGS+=(-e "autonomy_play_mode=${MODE}") ;;
    restart) EXTRA_ARGS+=(-e "autonomy_play_mode=restart") ;;
    *)
      log_message error "Unknown mode: ${MODE}"
      usage
      exit 1
    ;;
  esac
  if [[ "${MODE}" == cross-* ]]; then
    RELEASE_CONFIG="${ROOT}/vars/release-cross.yml"
  fi
}

# Resolve the environment argument to INVENTORY and PLAYBOOK. Reject unsupported release targets.
configure_inventory() {
  local environment="$1"

  if [[ -n "${environment}" ]]; then
    if [[ ! "${environment}" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ||
          ! -f "inventory/${environment}/hosts.yml" || -L "inventory/${environment}" ||
          -L "inventory/${environment}/hosts.yml" ]]; then
      log_message error "Unknown or non-canonical inventory: ${environment}"
      exit 1
    fi
    INVENTORY="inventory/${environment}/hosts.yml"
  fi

  if [[ ( "${MODE}" == "deploy" || "${MODE}" == "release" || "${MODE}" == "cross-release" ) && "${INVENTORY}" == "inventory/hosts.yml" ]]; then
    INVENTORY="inventory/robots/hosts.yml"
  fi

  if [[ "${INVENTORY}" == "inventory/production/hosts.yml" ]]; then
    case "${MODE}" in
      full|build|release|cross-release)
        log_message error "Production must promote an approved staging artifact through the release workflow."
        exit 1
        ;;
    esac
    EXTRA_ARGS+=(-e "autonomy_profile=production" -e "use_locked_versions=true" -e "autonomy_require_ci=true")
  fi

  if [[ "${MODE}" == "cross-build" && "${INVENTORY}" != "inventory/hosts.yml" ]]; then
    log_message error "cross-build uses the controller and does not take an inventory."
    exit 1
  fi
  if [[ "${MODE}" == "cross-release" && "${INVENTORY}" != "inventory/robots/hosts.yml" &&
        "${INVENTORY}" != "inventory/staging/hosts.yml" ]]; then
    log_message error "cross-release accepts robots or staging inventory."
    exit 1
  fi

  case "${INVENTORY}" in
    inventory/production/hosts.yml) PLAYBOOK="playbooks/production.yml" ;;
    inventory/staging/hosts.yml) PLAYBOOK="playbooks/staging.yml" ;;
    inventory/sim/hosts.yml) PLAYBOOK="playbooks/sim.yml" ;;
  esac
}

# Parse only deploy.sh options. Keep forwarded tokens in ANSIBLE_ARGS in their original order.
parse_options() {
  while [[ $# -gt 0 ]]; do
    # Normalize wrapper aliases first so every spelling uses the same validation.
    case "$1" in
      -R) set -- --release-config "${@:2}" ;;
      -I) set -- --incremental "${@:2}" ;;
      -m) set -- --module "${@:2}" ;;
      -B) set -- --base-artifact "${@:2}" ;;
      -S) set -- --test "${@:2}" ;;
      --release-config=*|--module=*|--base-artifact=*)
        set -- "${1%%=*}" "${1#*=}" "${@:2}"
        ;;
    esac
    case "$1" in
      --release-config)
        if [[ ( "${MODE}" != "release" && "${MODE}" != "plan" && "${MODE}" != cross-* ) ||
              -z "${2:-}" || "$2" == -* ]]; then
          log_message error "--release-config requires release, plan or cross mode and a file path"
          exit 1
        fi
        RELEASE_CONFIG="$2"
        explicit_release_config=true
        shift 2
        ;;
      --incremental)
        case "${MODE}" in
          deploy|release|cross-release) EXTRA_ARGS+=(-e "autonomy_artifact_incremental=true") ;;
          build|cross-build) ;;
          *)
            log_message error "--incremental requires build, cross-build, deploy, release or cross-release."
            exit 1
            ;;
        esac
        incremental=true
        shift
        ;;
      --test)
        if [[ "${MODE}" != "build" ]]; then
          log_message error "--test requires native build mode."
          exit 1
        fi
        EXTRA_ARGS+=(-e autonomy_build_test=true)
        shift
        ;;
      --module)
        if [[ "${MODE}" != "build" && "${MODE}" != "cross-build" && "${MODE}" != "cross-release" && "${MODE}" != "plan" ]]; then
          log_message error "--module requires build, cross-build, cross-release or plan."
          exit 1
        fi
        if [[ ! "${2:-}" =~ ^[a-z][a-z0-9_]*(/[a-z][a-z0-9_]*)?$ ]]; then
          log_message error "--module requires a project or autonomy/<domain>."
          exit 1
        fi
        modules+=("$2")
        module_args+=(--module "$2")
        shift 2
        ;;
      --base-artifact)
        if [[ ( "${MODE}" != "cross-build" && "${MODE}" != "cross-release" && "${MODE}" != "plan" ) ||
              -z "${2:-}" || "$2" == -* ]]; then
          log_message error "--base-artifact requires cross-build, cross-release or plan and an archive path."
          exit 1
        fi
        base_artifact="$2"
        shift 2
        ;;
      --check|-C)
        if [[ "${MODE}" == "cross-release" ]]; then
          log_message error "For check mode, use 'deploy --check' with a previously built artifact."
          exit 1
        fi
        prepare_release=false
        ANSIBLE_ARGS+=("$1")
        shift
        ;;
      --syntax-check|--list-*)
        prepare_release=false
        ANSIBLE_ARGS+=("$1")
        shift
        ;;
      -i|-i?*|--inventory|--inventory=*|--inventory-file|--inventory-file=*)
        log_message error "Select the inventory by its canonical name after the mode."
        exit 1
        ;;
      --start-at-task|--start-at-task=*|--skip-tags|--skip-tags=*|--tags|--tags=*|-t|-t?*|--step)
        case "${MODE}" in
          build|deploy|release|cross-release|plan|verify|push|restart)
            log_message error "Task selectors are not supported by managed builds, releases, preflight or maintenance."
            exit 1
            ;;
        esac
        ANSIBLE_ARGS+=("$1")
        shift
        ;;
      -l|--limit)
        if [[ $# -gt 1 && -n "$2" && "$2" != -* ]]; then
          cross_limit="$2"
          ANSIBLE_ARGS+=("$1" "$2")
          shift 2
        else
          log_message error "$1 requires a host or group pattern."
          exit 1
        fi
        ;;
      -l?*) cross_limit="${1:2}"; ANSIBLE_ARGS+=("$1"); shift ;;
      --limit=?*) cross_limit="${1#--limit=}"; ANSIBLE_ARGS+=("$1"); shift ;;
      *) ANSIBLE_ARGS+=("$1"); shift ;;
    esac
  done
}

# Validate the parsed options before contacting hosts. Populate release_args for the builder.
validate_options() {
  if [[ -n "${base_artifact}" && ! ${modules[0]+set} ]] ||
     [[ ( "${MODE}" == "cross-release" || "${MODE}" == "plan" ) && ${modules[0]+set} && -z "${base_artifact}" ]]; then
    log_message error "Module deployment requires both --module and --base-artifact."
    exit 1
  fi
  if [[ -n "${base_artifact}" ]]; then
    if [[ ! -f "${base_artifact}" ]]; then
      log_message error "Base artifact does not exist: ${base_artifact}"
      exit 1
    fi
    incremental=true
    module_plan_args=(--cross --base-artifact "${base_artifact}" "${module_args[@]}")
    if [[ "${MODE}" == "plan" && "${explicit_release_config}" == "false" ]]; then
      RELEASE_CONFIG="${ROOT}/vars/release-cross.yml"
      explicit_release_config=true
    fi
    if [[ "${MODE}" == "cross-release" ]]; then
      EXTRA_ARGS+=(-e autonomy_artifact_incremental=true -e autonomy_require_health_probe=true)
    fi
  fi

  if [[ "${MODE}" == "cross-build" && ${ANSIBLE_ARGS[0]+set} ]]; then
    log_message error "cross-build accepts --release-config, --incremental, --module and --base-artifact."
    exit 1
  fi
  if [[ "${MODE}" == "cross-release" && "${prepare_release}" == "true" && -z "${cross_limit}" ]]; then
    log_message error "Select a host or group with --limit for cross-release."
    exit 1
  fi

  release_args=(--config "${RELEASE_CONFIG}")
  if [[ "${incremental}" == "true" ]]; then
    release_args+=(--incremental)
  fi
  if [[ -n "${base_artifact}" ]]; then
    release_args+=(--base-artifact "${base_artifact}")
  fi
}

# Prepare the local Python environment when needed and hand off this controller-only build.
run_cross_build() {
  if ! python3 -c 'import yaml' 2>/dev/null; then
    source "${ROOT}/scripts/controller_env.sh"
  fi
  exec python3 "${ROOT}/scripts/prepare_release.py" --cross "${release_args[@]}" ${module_args[@]+"${module_args[@]}"}
}

# Ensure Ansible and its collection are available. Add vault and sudo options to EXTRA_ARGS.
prepare_controller() {
  local arg ask_become

  if ! command -v ansible-playbook >/dev/null 2>&1; then
    source "${ROOT}/scripts/controller_env.sh"
  fi

  if [[ "${MODE}" == "build" && ${modules[0]+set} ]]; then
    EXTRA_ARGS+=(-e "$(python3 -c 'import json, sys; print(json.dumps({"autonomy_build_modules": sys.argv[1:]}))' "${modules[@]}")")
  fi

  if [[ -f "${ROOT}/.vault_pass" ]]; then
    EXTRA_ARGS+=(--vault-password-file "${ROOT}/.vault_pass")
  fi

  if [[ "${INVENTORY}" == "inventory/hosts.yml" && "${MODE}" != "check" &&
        "${MODE}" != "plan" && "${MODE}" != "verify" && -t 0 ]]; then
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

  "${ROOT}/scripts/install_collection.sh"
}

# Add the builder contract to EXTRA_ARGS for a standalone plan with a release config.
describe_requested_plan() {
  local target_vars
  if [[ "${MODE}" == "plan" && "${explicit_release_config}" == "true" ]]; then
    target_vars="$(python3 "${ROOT}/scripts/prepare_release.py" --describe --config "${RELEASE_CONFIG}" \
      ${module_plan_args[@]+"${module_plan_args[@]}"})"
    EXTRA_ARGS+=(-e "autonomy_deploy_mode=artifact" -e "${target_vars}")
  fi
}

# Check the exact inventory and limit before any remote work.
require_selected_targets() {
  local selected
  # Keep the internally parsed host listing plain, even when terminal colors are forced.
  selected="$(ANSIBLE_FORCE_COLOR=0 ANSIBLE_NOCOLOR=1 \
    ansible-playbook -i "${INVENTORY}" "${PLAYBOOK}" "${EXTRA_ARGS[@]}" "$@" --list-hosts)"
  if [[ ! "${selected}" =~ hosts\ \([1-9][0-9]*\): ]]; then
    log_message error "Host selector matched no ${MODE} targets."
    exit 1
  fi
}

# Use the supplied Ansible options for preflight. Append artifact variables only after successful packaging.
prepare_build_or_release() {
  local -a build_kind=() target_args=(-e autonomy_play_mode=plan)
  local target_vars release_vars

  if [[ "${MODE}" == "cross-release" ]]; then
    build_kind+=(--cross)
  fi
  if [[ "${MODE}" != "build" ]]; then
    target_vars="$(python3 "${ROOT}/scripts/prepare_release.py" ${build_kind[@]+"${build_kind[@]}"} \
      --describe --config "${RELEASE_CONFIG}" ${module_plan_args[@]+"${module_plan_args[@]}"})"
    target_args+=(-e "autonomy_deploy_mode=artifact" -e "${target_vars}")
  fi

  log_message info "==> Preflight selected hosts before building"
  ansible-playbook -i "${INVENTORY}" "${PLAYBOOK}" "${EXTRA_ARGS[@]}" "$@" "${target_args[@]}"
  if [[ "${MODE}" != "build" ]]; then
    release_vars="$(python3 "${ROOT}/scripts/prepare_release.py" ${build_kind[@]+"${build_kind[@]}"} \
      "${release_args[@]}" ${module_args[@]+"${module_args[@]}"})"
    EXTRA_ARGS+=(-e "${release_vars}")
  fi
}

# Check host selection and dispatch the preflight before the final playbook invocation.
prepare_targets() {
  local -a plan_args=()
  local arg
  if [[ "${prepare_release}" != "true" ]]; then
    return 0
  fi

  describe_requested_plan
  # Module preflight reads a protected checkpoint and may need the sudo prompt.
  for arg in "$@"; do
    if [[ -n "${base_artifact}" || ( "${arg}" != "-K" && "${arg}" != "--ask-become-pass" ) ]]; then
      plan_args+=("${arg}")
    fi
  done
  case "${MODE}" in
    deploy|plan|verify|build|release|cross-release)
      require_selected_targets ${plan_args[@]+"${plan_args[@]}"}
      ;;
  esac
  case "${MODE}" in
    release|cross-release|build)
      prepare_build_or_release ${plan_args[@]+"${plan_args[@]}"}
      ;;
  esac
}

# The subshell owns the shared delta cache and its EXIT trap, including on Ansible failure.
run_incremental_deployment() (
  delta_cache="$(mktemp -d "${TMPDIR:-/tmp}/autonomy-delta.XXXXXXXX")"
  trap 'rm -rf -- "${delta_cache}"' EXIT
  ansible-playbook -i "${INVENTORY}" "${PLAYBOOK}" "$@" "${EXTRA_ARGS[@]}" \
    -e "autonomy_delta_cache_dir=${delta_cache}"
)

# Run the selected playbook and propagate Ansible’s exit status.
run_deployment() {
  log_message info "==> mode=${MODE} inventory=${INVENTORY} playbook=${PLAYBOOK}"
  if [[ "${prepare_release}" == "true" && "${incremental}" == "true" ]]; then
    case "${MODE}" in
      deploy|release|cross-release)
        run_incremental_deployment "$@"
        return
        ;;
    esac
  fi

  # Preserve the existing ordering of operator overrides and deployment variables.
  if [[ "${prepare_release}" == "true" &&
        ( "${MODE}" == "release" || "${MODE}" == "cross-release" ||
          "${MODE}" == "plan" || "${MODE}" == "verify" || "${INVENTORY}" == "inventory/production/hosts.yml" ) ]]; then
    exec ansible-playbook -i "${INVENTORY}" "${PLAYBOOK}" "$@" "${EXTRA_ARGS[@]}"
  fi
  exec ansible-playbook -i "${INVENTORY}" "${PLAYBOOK}" "${EXTRA_ARGS[@]}" "$@"
}

# Dispatch local-only modes, then prepare and run Ansible for deployment modes.
main() {
  # Keep command state local to this invocation. Helpers use Bash's dynamic
  # scoping to read or update it, without namerefs that require newer Bash.
  local MODE=full RELEASE_CONFIG="${ROOT}/vars/release.yml"
  local PLAYBOOK=playbooks/site.yml INVENTORY=inventory/hosts.yml
  local -a EXTRA_ARGS=() ANSIBLE_ARGS=() command_args=()
  local -a modules=() module_args=() module_plan_args=() release_args=()
  local base_artifact="" cross_limit="" environment=""
  local prepare_release=true incremental=false explicit_release_config=false
  local log_color="${AUTONOMY_LOG_COLOR:-auto}"

  cd "${ROOT}"
  configure_log_color "$@"
  set -- ${command_args[@]+"${command_args[@]}"}
  if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
  fi

  case "${1:-}" in
    full|build|deploy|release|cross-build|cross-release|push|check|plan|verify|restart|style)
      MODE="$1"
      shift
    ;;
  esac

  if [[ "${MODE}" == "style" ]]; then
    exec python3 "${ROOT}/../tools/python/clang_format_sources.py" --check "$@"
  fi

  if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
  fi

  if [[ $# -gt 0 && "$1" != -* ]]; then
    environment="$1"
    shift
  elif [[ "${MODE}" != "cross-build" ]]; then
    environment="${AUTONOMY_DEPLOY_ENV:-}"
  fi

  configure_mode
  configure_inventory "${environment}"
  parse_options "$@"
  validate_options
  if [[ "${MODE}" == "cross-build" ]]; then
    run_cross_build
  fi

  prepare_controller ${ANSIBLE_ARGS[@]+"${ANSIBLE_ARGS[@]}"}
  prepare_targets ${ANSIBLE_ARGS[@]+"${ANSIBLE_ARGS[@]}"}
  run_deployment ${ANSIBLE_ARGS[@]+"${ANSIBLE_ARGS[@]}"}
}

main "$@"
