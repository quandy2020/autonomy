#!/usr/bin/env bash
#
# automsgs.sh — one-shot CLI for the automsgs Bazel workspace.
#
# Quick start:
#   ./automsgs.sh help
#   ./automsgs.sh build
#   ./automsgs.sh test
#   ./automsgs.sh install --prefix ./install
#   ./automsgs.sh all
#   ./automsgs.sh clean
#
# Protobuf (protoc, libprotobuf, libprotoc) is expected under /usr/local,
# matching the Autolink SDK image.
#
set -euo pipefail

readonly SCRIPT_NAME="$(basename "${BASH_SOURCE[0]}")"
readonly ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "${ROOT}"

BAZEL="${BAZEL:-bazel}"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 8)}"
PREFIX="${PREFIX:-${ROOT}/install}"
VERBOSE="${VERBOSE:-0}"

readonly BUILD_TARGETS=(
  //:automsgs
  //:automsgs_py
  //:automsgs_msgs
  //:automsgs_protoc_plugin
  //:automsgs_protoc_plugin_lite
)

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

die()  { echo "${SCRIPT_NAME}: error: $*" >&2; exit 1; }
info() { echo "==> $*"; }
ok()   { echo "ok  $*"; }
note() { echo "    $*"; }

# ---------------------------------------------------------------------------
# Help
# ---------------------------------------------------------------------------

usage_summary() {
  cat <<EOF
Usage: ${SCRIPT_NAME} [global options] <command> [command options]

Commands:
  build       Compile libautomsgs, Python messages, CLI, and protoc plugins
  install     Build (if needed) and install into PREFIX
  test        Build, smoke-check artifacts, run bazel tests
  clean       Wipe the Bazel cache (and ./install when PREFIX is default)
  all         Run test, then install
  status      Workspace / toolchain / artifact summary
  help        Show this help, or help for a command

Global options:
  -j, --jobs N         Bazel parallel jobs          [default: ${JOBS}]
      --prefix DIR     Install prefix               [default: ${PREFIX}]
      --bazel PATH     Bazel binary                 [default: ${BAZEL}]
  -v, --verbose        Print underlying bazel commands
  -h, --help           Same as: ${SCRIPT_NAME} help

Examples:
  ${SCRIPT_NAME} build
  ${SCRIPT_NAME} -j 16 build
  ${SCRIPT_NAME} test
  ${SCRIPT_NAME} install --prefix /usr/local
  ${SCRIPT_NAME} all --prefix ${ROOT}/install

Environment:
  PREFIX   JOBS   BAZEL   VERBOSE
EOF
}

usage_build() {
  cat <<EOF
Usage: ${SCRIPT_NAME} build [global options]

Build:

  ${BUILD_TARGETS[*]}

Artifacts land under bazel-bin/:

  libautomsgs.so
  automsgs_msgs
  automsgs_protoc_plugin
  automsgs_protoc_plugin_lite
  python/automsgs/**/*_pb2.py
EOF
}

usage_install() {
  cat <<EOF
Usage: ${SCRIPT_NAME} install [global options] [--prefix DIR]

Build (if needed), then install into PREFIX.

Layout:
  PREFIX/lib/libautomsgs.so
  PREFIX/bin/automsgs-msgs
  PREFIX/bin/automsgs_protoc_plugin
  PREFIX/bin/automsgs_protoc_plugin_lite
  PREFIX/bin/automsgs_generate.py
  PREFIX/include/automsgs/**
  PREFIX/lib/python/automsgs/**
  PREFIX/share/automsgs/proto/**

After install:
  export LD_LIBRARY_PATH=PREFIX/lib:\$LD_LIBRARY_PATH
  export PATH=PREFIX/bin:\$PATH
  export PYTHONPATH=PREFIX/lib/python:\$PYTHONPATH
EOF
}

usage_test() {
  cat <<EOF
Usage: ${SCRIPT_NAME} test [global options]

1. Build core targets
2. Smoke-check bazel-bin/{libautomsgs.so,automsgs_msgs}
3. bazel test //...
EOF
}

usage_clean() {
  cat <<EOF
Usage: ${SCRIPT_NAME} clean [global options]

Runs: bazel clean --expunge

Also removes <workspace>/install when PREFIX still points at the default
local install directory. A custom --prefix is left in place.
EOF
}

usage_all() {
  cat <<EOF
Usage: ${SCRIPT_NAME} all [global options] [--prefix DIR]

Full local pipeline: test, then install.
EOF
}

usage_status() {
  cat <<EOF
Usage: ${SCRIPT_NAME} status

Print workspace path, Bazel version, JOBS/PREFIX, and whether artifacts exist.
EOF
}

usage_command() {
  case "${1:-}" in
    build)   usage_build ;;
    install) usage_install ;;
    test)    usage_test ;;
    clean)   usage_clean ;;
    all)     usage_all ;;
    status)  usage_status ;;
    help|"") usage_summary ;;
    *)
      usage_summary >&2
      die "no help for unknown command: $1"
      ;;
  esac
}

# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

require_bazel() {
  command -v "${BAZEL}" >/dev/null 2>&1 \
    || die "'${BAZEL}' not found in PATH (set --bazel or BAZEL=)"
  [[ -f "${ROOT}/MODULE.bazel" ]] \
    || die "not an automsgs Bazel workspace: ${ROOT}"
}

run_bazel() {
  if [[ "${VERBOSE}" == "1" ]]; then
    info "+ ${BAZEL} $*"
  fi
  "${BAZEL}" "$@"
}

bazel_bin_dir() {
  if [[ -d "${ROOT}/bazel-bin" ]]; then
    printf '%s\n' "${ROOT}/bazel-bin"
  else
    run_bazel info bazel-bin
  fi
}

parse_no_args() {
  local usage_fn="$1"
  shift
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help)
        "${usage_fn}"
        return 1
        ;;
      *)
        die "${usage_fn#usage_}: unexpected argument: $1 (try --help)"
        ;;
    esac
  done
  return 0
}

# Sets CONSUMED to the number of argv entries used (0 if $1 is not --prefix).
consume_prefix() {
  CONSUMED=0
  case "${1:-}" in
    --prefix)
      [[ $# -ge 2 ]] || die "--prefix needs a directory"
      PREFIX="$2"
      CONSUMED=2
      ;;
    --prefix=*)
      PREFIX="${1#--prefix=}"
      CONSUMED=1
      ;;
  esac
}

# ---------------------------------------------------------------------------
# Build / test
# ---------------------------------------------------------------------------

cmd_build() {
  parse_no_args usage_build "$@" || return 0
  require_bazel
  info "build (${JOBS} jobs)"
  note "${BUILD_TARGETS[*]}"
  run_bazel build --jobs="${JOBS}" "${BUILD_TARGETS[@]}"
  ok "build → $(bazel_bin_dir)"
}

smoke_library() {
  local lib="$1"
  [[ -f "${lib}" ]] || die "missing library: ${lib}"
  info "smoke: libautomsgs.so"
  file "${lib}" | grep -qi 'shared object' \
    || die "${lib} is not a shared object"
}

smoke_cli() {
  local cli="$1"
  [[ -x "${cli}" ]] || die "missing executable: ${cli}"
  info "smoke: automsgs_msgs --help"
  "${cli}" --help >/dev/null
}

cmd_test() {
  parse_no_args usage_test "$@" || return 0
  require_bazel
  cmd_build

  local bin
  bin="$(bazel_bin_dir)"
  smoke_library "${bin}/libautomsgs.so"
  smoke_cli "${bin}/automsgs_msgs"
  info "bazel test //..."
  run_bazel test --jobs="${JOBS}" //...
  ok "test passed"
}

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install_tree() {
  local src="$1"
  local dest="$2"
  [[ -d "${src}" ]] || return 0
  mkdir -p "${dest}"
  (
    cd "${src}"
    find . -type f -print0
  ) | while IFS= read -r -d '' rel; do
    mkdir -p "${dest}/$(dirname "${rel}")"
    install -m 644 "${src}/${rel#./}" "${dest}/${rel#./}"
  done
}

install_headers() {
  local bin="$1"
  local dest="$2"
  local inc="${dest}/include"
  if command -v rsync >/dev/null 2>&1; then
    mkdir -p "${inc}"
    rsync -a \
      --include='*/' \
      --include='*.hh' \
      --include='*.hpp' \
      --include='*.h' \
      --exclude='*' \
      "${ROOT}/core/include/automsgs/" "${inc}/automsgs/"
  else
    install_tree "${ROOT}/core/include/automsgs" "${inc}/automsgs"
  fi
  if [[ -d "${bin}/automsgs" ]]; then
    (
      cd "${bin}"
      find automsgs -type f \( -name '*.h' -o -name '*.hh' -o -name '*.hpp' \) -print0
    ) | while IFS= read -r -d '' rel; do
      mkdir -p "${inc}/$(dirname "${rel}")"
      install -m 644 "${bin}/${rel}" "${inc}/${rel}"
    done
  fi
}

install_python() {
  local bin="$1"
  local dest="$2"
  local py="${dest}/lib/python/automsgs"
  if [[ -d "${bin}/python/automsgs" ]]; then
    rm -rf "${py}"
    mkdir -p "${dest}/lib/python"
    cp -a "${bin}/python/automsgs" "${dest}/lib/python/"
  fi
  if [[ -f "${ROOT}/python/src/__init__.py" ]]; then
    mkdir -p "${py}"
    install -m 644 "${ROOT}/python/src/__init__.py" "${py}/__init__.py"
  fi
}

install_protos() {
  local dest="$1/share/automsgs/proto"
  mkdir -p "${dest}"
  (
    cd "${ROOT}/proto"
    find msgs srvs rpcs actions task -name '*.proto' -print0
  ) | while IFS= read -r -d '' rel; do
    mkdir -p "${dest}/$(dirname "${rel}")"
    install -m 644 "${ROOT}/proto/${rel}" "${dest}/${rel}"
  done
}

parse_install_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help)
        usage_install
        return 1
        ;;
      --prefix|--prefix=*)
        consume_prefix "$@"
        [[ "${CONSUMED}" -gt 0 ]] || die "install: bad --prefix"
        shift "${CONSUMED}"
        ;;
      *)
        die "install: unexpected argument: $1 (try --help)"
        ;;
    esac
  done
  return 0
}

cmd_install() {
  parse_install_args "$@" || return 0
  require_bazel
  cmd_build

  local bin dest
  bin="$(bazel_bin_dir)"
  dest="${PREFIX}"
  [[ -d "${bin}" ]] || die "bazel-bin missing; build failed?"

  info "install → ${dest}"
  mkdir -p "${dest}/lib" "${dest}/bin" "${dest}/include" "${dest}/share/automsgs/proto"

  install -m 755 "${bin}/libautomsgs.so" "${dest}/lib/libautomsgs.so"
  install -m 755 "${bin}/automsgs_msgs" "${dest}/bin/automsgs-msgs"
  install -m 755 "${bin}/automsgs_protoc_plugin" "${dest}/bin/automsgs_protoc_plugin"
  install -m 755 "${bin}/automsgs_protoc_plugin_lite" "${dest}/bin/automsgs_protoc_plugin_lite"
  install -m 755 "${ROOT}/tools/automsgs_msgs_generate.py" "${dest}/bin/automsgs_generate.py"
  install -m 755 "${ROOT}/tools/automsgs_generate_factory.py" "${dest}/bin/automsgs_generate_factory.py"
  if [[ -f "${ROOT}/tools/cli/rpc-cli.py" ]]; then
    install -m 755 "${ROOT}/tools/cli/rpc-cli.py" "${dest}/bin/rpc-cli"
  fi

  install_headers "${bin}" "${dest}"
  install_python "${bin}" "${dest}"
  install_protos "${dest}"

  ok "installed under ${dest}"
  note "export LD_LIBRARY_PATH=${dest}/lib:\${LD_LIBRARY_PATH}"
  note "export PATH=${dest}/bin:\${PATH}"
  note "export PYTHONPATH=${dest}/lib/python:\${PYTHONPATH}"
}

# ---------------------------------------------------------------------------
# Clean / all / status
# ---------------------------------------------------------------------------

cmd_clean() {
  parse_no_args usage_clean "$@" || return 0
  require_bazel
  info "bazel clean --expunge"
  run_bazel clean --expunge
  if [[ -d "${ROOT}/install" && "${PREFIX}" == "${ROOT}/install" ]]; then
    info "remove default install tree: ${ROOT}/install"
    rm -rf "${ROOT}/install"
  fi
  ok "clean complete"
}

cmd_all() {
  local -a install_args=()
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --prefix|--prefix=*)
        install_args+=("$1")
        if [[ "$1" == "--prefix" ]]; then
          [[ $# -ge 2 ]] || die "--prefix needs a directory"
          install_args+=("$2")
          shift 2
        else
          shift
        fi
        ;;
      -h|--help) usage_all; return 0 ;;
      *) die "all: unexpected argument: $1 (try --help)" ;;
    esac
  done
  cmd_test
  cmd_install "${install_args[@]+"${install_args[@]}"}"
}

status_print_config() {
  echo "workspace : ${ROOT}"
  echo "bazel     : ${BAZEL} ($(command -v "${BAZEL}" 2>/dev/null || echo 'not found'))"
  if command -v "${BAZEL}" >/dev/null 2>&1; then
    echo "version   : $(${BAZEL} --version 2>/dev/null | head -1 || echo unknown)"
  fi
  echo "jobs      : ${JOBS}"
  echo "prefix    : ${PREFIX}"
  echo "verbose   : ${VERBOSE}"
}

status_print_bazel_bin() {
  local bin=""
  [[ -d "${ROOT}/bazel-bin" ]] && bin="${ROOT}/bazel-bin"
  echo "bazel-bin : ${bin:-'(not built yet)'}"
  [[ -n "${bin}" ]] || return 0
  local f
  for f in libautomsgs.so automsgs_msgs automsgs_protoc_plugin automsgs_protoc_plugin_lite; do
    if [[ -e "${bin}/${f}" ]]; then
      echo "  artifact: ${f}  ($(du -h "${bin}/${f}" | awk '{print $1}'))"
    else
      echo "  missing : ${f}"
    fi
  done
}

status_print_prefix() {
  if [[ ! -d "${PREFIX}" ]]; then
    echo "install   : (no PREFIX directory yet)"
    return 0
  fi
  echo "install   : present"
  local f
  for f in lib/libautomsgs.so bin/automsgs-msgs bin/automsgs_generate.py; do
    if [[ -e "${PREFIX}/${f}" ]]; then
      echo "  present : ${f}"
    else
      echo "  missing : ${f}"
    fi
  done
}

cmd_status() {
  parse_no_args usage_status "$@" || return 0
  status_print_config
  status_print_bazel_bin
  status_print_prefix
}

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

PARSE_SHIFT=0
PARSE_HELP=0

parse_global_flag() {
  PARSE_SHIFT=0
  PARSE_HELP=0
  case "$1" in
    -j|--jobs)
      [[ $# -ge 2 ]] || die "$1 needs a value"
      JOBS="$2"
      PARSE_SHIFT=2
      ;;
    --jobs=*)
      JOBS="${1#--jobs=}"
      PARSE_SHIFT=1
      ;;
    --prefix|--prefix=*)
      consume_prefix "$@"
      [[ "${CONSUMED}" -gt 0 ]] || return 1
      PARSE_SHIFT="${CONSUMED}"
      ;;
    --bazel)
      [[ $# -ge 2 ]] || die "--bazel needs a path"
      BAZEL="$2"
      PARSE_SHIFT=2
      ;;
    --bazel=*)
      BAZEL="${1#--bazel=}"
      PARSE_SHIFT=1
      ;;
    -v|--verbose)
      VERBOSE=1
      PARSE_SHIFT=1
      ;;
    -h|--help)
      PARSE_SHIFT=1
      PARSE_HELP=1
      ;;
    *)
      return 1
      ;;
  esac
  return 0
}

dispatch_command() {
  local cmd="$1"
  shift
  case "${cmd}" in
    build)   cmd_build "$@" ;;
    install) cmd_install "$@" ;;
    test)    cmd_test "$@" ;;
    clean)   cmd_clean "$@" ;;
    all)     cmd_all "$@" ;;
    status)  cmd_status "$@" ;;
    help|-h|--help)
      if [[ $# -eq 0 ]]; then
        usage_summary
      else
        usage_command "$1"
      fi
      ;;
    *)
      usage_summary >&2
      die "unknown command: ${cmd}"
      ;;
  esac
}

main() {
  local cmd=""
  local -a cmd_args=()

  while [[ $# -gt 0 ]]; do
    if parse_global_flag "$@"; then
      if [[ "${PARSE_HELP}" == "1" && $# -eq "${PARSE_SHIFT}" ]]; then
        usage_summary
        return 0
      fi
      shift "${PARSE_SHIFT}"
      continue
    fi
    break
  done

  cmd="${1:-help}"
  shift || true

  while [[ $# -gt 0 ]]; do
    if parse_global_flag "$@"; then
      if [[ "${PARSE_HELP}" == "1" ]]; then
        usage_command "${cmd}"
        return 0
      fi
      shift "${PARSE_SHIFT}"
      continue
    fi
    cmd_args+=("$1")
    shift
  done

  dispatch_command "${cmd}" "${cmd_args[@]+"${cmd_args[@]}"}"
}

main "$@"
