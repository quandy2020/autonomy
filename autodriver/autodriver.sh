#!/usr/bin/env bash
#
# autodriver.sh — one-shot CLI for the autodriver Bazel workspace.
#
# Quick start:
#   ./autodriver.sh help
#   ./autodriver.sh build
#   ./autodriver.sh test
#   ./autodriver.sh install --prefix ./install
#   ./autodriver.sh all
#   ./autodriver.sh clean
#
# Requires a CMake prefix with libautolink.so + libautomsgs.so:
#   export AUTONOMY_PREFIX=$PWD/../../../install/autonomy
#
set -euo pipefail

readonly SCRIPT_NAME="$(basename "${BASH_SOURCE[0]}")"
readonly ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "${ROOT}"

BAZEL="${BAZEL:-bazel}"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 8)}"
PREFIX="${PREFIX:-${ROOT}/install}"
VERBOSE="${VERBOSE:-0}"

# Default CMake prefix search (monorepo install → build → env).
if [[ -z "${AUTONOMY_PREFIX:-}" ]]; then
  for candidate in \
    "${ROOT}/../../../install/autonomy" \
    "${ROOT}/../../../build/autonomy" \
    "${ROOT}/../install" \
    "${ROOT}/install"
  do
    if [[ -f "${candidate}/lib/libautolink.so" ]]; then
      AUTONOMY_PREFIX="$(cd "${candidate}" && pwd)"
      break
    fi
  done
fi
export AUTONOMY_PREFIX="${AUTONOMY_PREFIX:-}"

readonly BUILD_TARGETS=(
  //:autodriver
  //:autodriver_bin
  //:autodriver_jetauto
  //:autodriver_l1w
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
  build       Compile libautodriver, binary, and chassis plugins
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

Environment:
  AUTONOMY_PREFIX   CMake prefix with libautolink.so   [now: ${AUTONOMY_PREFIX:-'(unset)'}]
  PREFIX   JOBS   BAZEL   VERBOSE

Examples:
  export AUTONOMY_PREFIX=\$PWD/../../../install/autonomy
  ${SCRIPT_NAME} build
  ${SCRIPT_NAME} -j 16 test
  ${SCRIPT_NAME} install --prefix /usr/local
  ${SCRIPT_NAME} all --prefix ${ROOT}/install
EOF
}

usage_build() {
  cat <<EOF
Usage: ${SCRIPT_NAME} build [global options]

Build:
  ${BUILD_TARGETS[*]}

Artifacts under bazel-bin/:
  libautodriver.so
  autodriver_bin
  libautodriver_jetauto.so
  libautodriver_l1w.so
EOF
}

usage_install() {
  cat <<EOF
Usage: ${SCRIPT_NAME} install [global options] [--prefix DIR]

Layout (PREFIX):
  lib/libautodriver.so
  lib/libautodriver_jetauto.so
  lib/libautodriver_l1w.so
  bin/autodriver
  include/autodriver/**
  include/chassis/**
  share/autodriver/{config,dag,launch}/

After install:
  export LD_LIBRARY_PATH=PREFIX/lib:\$LD_LIBRARY_PATH
  export PATH=PREFIX/bin:\$PATH
  export AUTODRIVER_PATH=PREFIX/share/autodriver
EOF
}

usage_test() {
  cat <<EOF
Usage: ${SCRIPT_NAME} test [global options]

1. Build core targets
2. Smoke-check bazel-bin artifacts
3. bazel test //...
EOF
}

usage_clean() {
  cat <<EOF
Usage: ${SCRIPT_NAME} clean

Runs: bazel clean --expunge
Also removes ${ROOT}/install when PREFIX is the default local tree.
EOF
}

usage_all() {
  cat <<EOF
Usage: ${SCRIPT_NAME} all [global options] [--prefix DIR]

Full pipeline: test && install
EOF
}

usage_status() {
  cat <<EOF
Usage: ${SCRIPT_NAME} status

Print workspace, Bazel version, AUTONOMY_PREFIX, JOBS/PREFIX, and artifacts.
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
    *) usage_summary >&2; die "no help for unknown command: $1" ;;
  esac
}

# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

require_bazel() {
  command -v "${BAZEL}" >/dev/null 2>&1 \
    || die "'${BAZEL}' not found in PATH (set --bazel or BAZEL=)"
  [[ -f "${ROOT}/MODULE.bazel" ]] \
    || die "not an autodriver Bazel workspace: ${ROOT}"
  [[ -n "${AUTONOMY_PREFIX}" && -f "${AUTONOMY_PREFIX}/lib/libautolink.so" ]] \
    || die "AUTONOMY_PREFIX missing libautolink.so (export AUTONOMY_PREFIX=.../install/autonomy)"
}

run_bazel() {
  if [[ "${VERBOSE}" == "1" ]]; then
    info "+ AUTONOMY_PREFIX=${AUTONOMY_PREFIX} ${BAZEL} $*"
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
      -h|--help) "${usage_fn}"; return 1 ;;
      *) die "${usage_fn#usage_}: unexpected argument: $1 (try --help)" ;;
    esac
  done
  return 0
}

parse_prefix_arg() {
  case "$1" in
    --prefix)
      [[ $# -ge 2 ]] || die "--prefix needs a directory"
      PREFIX="$2"
      return 2
      ;;
    --prefix=*)
      PREFIX="${1#--prefix=}"
      return 1
      ;;
    *) return 0 ;;
  esac
}

# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

cmd_build() {
  parse_no_args usage_build "$@" || return 0
  require_bazel
  info "build (${JOBS} jobs)  prefix=${AUTONOMY_PREFIX}"
  note "${BUILD_TARGETS[*]}"
  # Ensure LD_LIBRARY_PATH covers the CMake .so tree for link/run.
  export LD_LIBRARY_PATH="${AUTONOMY_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  run_bazel build --jobs="${JOBS}" "${BUILD_TARGETS[@]}"
  ok "build → $(bazel_bin_dir)"
}

smoke_file() {
  local path="$1" kind="$2"
  [[ -e "${path}" ]] || die "missing ${kind}: ${path}"
  case "${kind}" in
    library)
      file "${path}" | grep -qi 'shared object' || die "${path} is not a shared object"
      ;;
    binary)
      [[ -x "${path}" ]] || die "not executable: ${path}"
      ;;
  esac
}

cmd_test() {
  parse_no_args usage_test "$@" || return 0
  require_bazel
  cmd_build

  local bin
  bin="$(bazel_bin_dir)"
  info "smoke artifacts"
  smoke_file "${bin}/libautodriver.so" library
  smoke_file "${bin}/autodriver_bin" binary
  smoke_file "${bin}/libautodriver_jetauto.so" library
  smoke_file "${bin}/libautodriver_l1w.so" library

  export LD_LIBRARY_PATH="${AUTONOMY_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  info "bazel test //..."
  run_bazel test --jobs="${JOBS}" //...
  ok "test passed"
}

_install_tree() {
  local src="$1" dest="$2"
  mkdir -p "${dest}"
  if command -v rsync >/dev/null 2>&1; then
    rsync -a --delete \
      --include='*/' \
      --include='*.hpp' --include='*.h' \
      --exclude='*' \
      "${src}/" "${dest}/"
    return
  fi
  rm -rf "${dest}"
  mkdir -p "${dest}"
  (cd "${src}" && find . -type f \( -name '*.hpp' -o -name '*.h' \) -print0 \
    | cpio -0pd "${dest}")
}

cmd_install() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help) usage_install; return 0 ;;
      --prefix|--prefix=*)
        local n=0
        parse_prefix_arg "$@" || n=$?
        [[ "${n}" -gt 0 ]] || die "install: bad --prefix"
        shift "${n}"
        ;;
      *) die "install: unexpected argument: $1 (try --help)" ;;
    esac
  done

  require_bazel
  cmd_build

  local bin dest
  bin="$(bazel_bin_dir)"
  dest="${PREFIX}"
  [[ -d "${bin}" ]] || die "bazel-bin missing; build failed?"

  info "install → ${dest}"
  mkdir -p \
    "${dest}/lib" \
    "${dest}/bin" \
    "${dest}/include" \
    "${dest}/share/autodriver/config" \
    "${dest}/share/autodriver/dag" \
    "${dest}/share/autodriver/launch"

  install -m 755 "${bin}/libautodriver.so" "${dest}/lib/libautodriver.so"
  install -m 755 "${bin}/libautodriver_jetauto.so" "${dest}/lib/libautodriver_jetauto.so"
  install -m 755 "${bin}/libautodriver_l1w.so" "${dest}/lib/libautodriver_l1w.so"
  install -m 755 "${bin}/autodriver_bin" "${dest}/bin/autodriver"

  _install_tree "${ROOT}/autodriver" "${dest}/include/autodriver"
  _install_tree "${ROOT}/chassis" "${dest}/include/chassis"

  if [[ -f "${bin}/conf/conf.hpp" ]]; then
    mkdir -p "${dest}/include/autodriver/conf"
    install -m 644 "${bin}/conf/conf.hpp" "${dest}/include/autodriver/conf/conf.hpp"
  fi

  if command -v rsync >/dev/null 2>&1; then
    rsync -a "${ROOT}/config/" "${dest}/share/autodriver/config/"
    rsync -a "${ROOT}/dag/" "${dest}/share/autodriver/dag/"
    rsync -a "${ROOT}/launch/" "${dest}/share/autodriver/launch/"
  else
    cp -a "${ROOT}/config/." "${dest}/share/autodriver/config/"
    cp -a "${ROOT}/dag/." "${dest}/share/autodriver/dag/"
    cp -a "${ROOT}/launch/." "${dest}/share/autodriver/launch/"
  fi

  ok "installed under ${dest}"
  note "export LD_LIBRARY_PATH=${dest}/lib:\${LD_LIBRARY_PATH}"
  note "export PATH=${dest}/bin:\${PATH}"
  note "export AUTODRIVER_PATH=${dest}/share/autodriver"
}

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
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --prefix|--prefix=*) break ;;
      -h|--help) usage_all; return 0 ;;
      *) die "all: unexpected argument: $1 (try --help)" ;;
    esac
  done
  cmd_test
  cmd_install "$@"
}

cmd_status() {
  parse_no_args usage_status "$@" || return 0
  echo "workspace : ${ROOT}"
  echo "bazel     : ${BAZEL} ($(command -v "${BAZEL}" 2>/dev/null || echo 'not found'))"
  if command -v "${BAZEL}" >/dev/null 2>&1; then
    echo "version   : $(${BAZEL} --version 2>/dev/null | head -1 || echo unknown)"
  fi
  echo "jobs      : ${JOBS}"
  echo "prefix    : ${PREFIX}"
  echo "autonomy  : ${AUTONOMY_PREFIX:-'(unset)'}"
  echo "verbose   : ${VERBOSE}"

  local bin=""
  [[ -d "${ROOT}/bazel-bin" ]] && bin="${ROOT}/bazel-bin"
  echo "bazel-bin : ${bin:-'(not built yet)'}"
  if [[ -n "${bin}" ]]; then
    local f
    for f in libautodriver.so autodriver_bin libautodriver_jetauto.so libautodriver_l1w.so; do
      if [[ -e "${bin}/${f}" ]]; then
        echo "  artifact: ${f}  ($(du -h "${bin}/${f}" | awk '{print $1}'))"
      else
        echo "  missing : ${f}"
      fi
    done
  fi

  if [[ ! -d "${PREFIX}" ]]; then
    echo "install   : (no PREFIX directory yet)"
  else
    echo "install   : present"
    local f
    for f in lib/libautodriver.so bin/autodriver; do
      if [[ -e "${PREFIX}/${f}" ]]; then
        echo "  present : ${f}"
      else
        echo "  missing : ${f}"
      fi
    done
  fi
}

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

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
      local n=0
      parse_prefix_arg "$@" || n=$?
      [[ "${n}" -gt 0 ]] || return 1
      PARSE_SHIFT="${n}"
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
    *) return 1 ;;
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
      if [[ $# -eq 0 ]]; then usage_summary; else usage_command "$1"; fi
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
